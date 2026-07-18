import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration as DurationMsg
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ik_bridge.forward_kinematics import forward_kinematics_point

# Formato de /ik_test/target_point y /trajectory_chunks (stride por punto):
# [rx, ry, rz, nx, ny, nz, theta] - debe coincidir con el firmware ESP32.
POINT_STRIDE = 7

# Formato de /ik_result_chunks: [q1, q2, q3, q4, q5, q6] por punto, ya en el
# orden natural de juntas del arm_controller (ver Cinematica_inversa_6DFODQ_test).
RESULT_STRIDE = 6

# Debe coincidir con MAX_POINTS_PER_CHUNK del firmware ESP32. Este es solo el
# tamano del buffer que reserva el firmware (malloc); NO es el limite real de
# cuantos puntos entran en un solo mensaje sobre la sesion micro-ROS/XRCE-DDS.
FIRMWARE_MAX_POINTS_PER_CHUNK = 100

# MTU real de la sesion micro-ROS/XRCE-DDS por UDP con el ESP32 (visto en el
# agente: "Trying to serialize N in 508 MTU stream"). Un Float32MultiArray de
# n puntos [rx,ry,rz,nx,ny,nz,theta] pesa n*POINT_STRIDE*4 + 16 bytes (16 =
# overhead fijo del mensaje, medido empiricamente: 2816 bytes para n=100 y
# 1416 bytes para n=50, ambos calzan con esa formula). Un chunk que supere
# el MTU no llega nunca al ESP32: el agente lo descarta en silencio (solo
# deja ese warning), asi que el firmware se queda repitiendo para siempre su
# ultimo punto valido y el brazo no se mueve.
_MICRO_ROS_MTU_BYTES = 508
_FLOAT32MULTIARRAY_OVERHEAD_BYTES = 16
MAX_POINTS_PER_CHUNK_FOR_MTU = (
    (_MICRO_ROS_MTU_BYTES - _FLOAT32MULTIARRAY_OVERHEAD_BYTES) // (POINT_STRIDE * 4))

# Default conservador para el MTU (deja margen respecto al limite exacto, ~17).
# Pero el limite real que importa para no generar saltos es OTRO: el ESP32
# drena su cola de trayectoria a 1 kHz, asi que un chunk de 15 puntos lo
# procesa entero en ~15 ms - muchisimo mas rapido de lo que tarda el brazo
# real/simulado en moverse a cada paso. Si se manda mas de 1 punto por
# chunk, el ESP32 termina de "recorrer" el chunk completo y se queda
# sosteniendo el ULTIMO punto de ese chunk antes de que /joint_states
# alcance a reflejar ninguno de los puntos intermedios: el filtro de saltos
# termina comparando contra puntos que ya quedaron muy atras. Por eso el
# default es 1 punto/chunk: cada punto se sostiene en el ESP32 hasta que
# llega el siguiente chunk, dandole tiempo real a /joint_states de alcanzarlo.
DEFAULT_POINTS_PER_CHUNK = 1

# Ritmo de publicacion de chunks. Con 1 punto/chunk, esto es literalmente
# "cada cuanto avanza el objetivo". Tiene que ser mas lento que el tiempo
# que tarda el arm_controller en ejecutar un paso (command_duration_sec) +
# el tiempo de asentamiento de la fisica/control, si no el ESP32 vuelve a
# adelantarse y el filtro de saltos rechaza todo igual que con chunks grandes.
DEFAULT_CHUNK_PUBLISH_RATE_HZ = 5.0

# Debe coincidir con el orden de "joints" en BRAZOROBOTICO_description/config/controllers.yaml.
DEFAULT_JOINT_NAMES = [
    'base_link_revolucion-5',
    'arm1_revolucion-6',
    'arm2_revolucion-7',
    'arm3_revolucion-8',
    'arm4_revolucion-9',
    'arm5_revolucion-11',
]

# Pose segura de referencia inicial (home), igual a la del firmware ESP32.
DEFAULT_HOME_POINT = [0.1, 0.05, 0.30, 1.0, 0.0, 0.0, -1.53]


class IKBridgeNode(Node):
    """Puente de prueba PC <-> ESP32 para validar la cinematica inversa.

    Recibe un punto deseado, lo interpola en chunks cartesianos y los manda
    al firmware ESP32 (/trajectory_chunks). El ESP32 calcula la IK a bordo y
    republica el resultado en /ik_result_chunks; este nodo sigue en todo
    momento la posicion real de las articulaciones via /joint_states, y solo
    reenvia al arm_controller los puntos de IK que no impliquen un salto de
    articulacion mayor al umbral configurado. Si detecta un salto, lo notifica
    y descarta ese punto puntual, pero sigue procesando el resto del flujo.
    """

    def __init__(self):
        super().__init__('ik_bridge_node')

        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('home_point', DEFAULT_HOME_POINT)
        self.declare_parameter('points_per_target', 150)
        self.declare_parameter('points_per_chunk', DEFAULT_POINTS_PER_CHUNK)
        self.declare_parameter('chunk_publish_rate_hz', DEFAULT_CHUNK_PUBLISH_RATE_HZ)
        self.declare_parameter('max_joint_jump_rad', 3.14)
        self.declare_parameter('command_duration_sec', 0.1)
        self.declare_parameter('arm_controller_topic', '/arm_controller/joint_trajectory')

        self._joint_names = list(self.get_parameter('joint_names').value)
        home = list(self.get_parameter('home_point').value)
        if len(home) != POINT_STRIDE:
            raise ValueError(f'home_point debe tener {POINT_STRIDE} floats, recibidos {len(home)}')

        self._points_per_target = int(self.get_parameter('points_per_target').value)

        points_per_chunk = int(self.get_parameter('points_per_chunk').value)
        if points_per_chunk > FIRMWARE_MAX_POINTS_PER_CHUNK:
            self.get_logger().warn(
                f'points_per_chunk={points_per_chunk} excede el limite del firmware '
                f'({FIRMWARE_MAX_POINTS_PER_CHUNK}). Se ajusta a {FIRMWARE_MAX_POINTS_PER_CHUNK}.')
            points_per_chunk = FIRMWARE_MAX_POINTS_PER_CHUNK
        if points_per_chunk > MAX_POINTS_PER_CHUNK_FOR_MTU:
            self.get_logger().warn(
                f'points_per_chunk={points_per_chunk} arma mensajes de '
                f'~{points_per_chunk * POINT_STRIDE * 4 + _FLOAT32MULTIARRAY_OVERHEAD_BYTES} '
                f'bytes, por encima del MTU de la sesion micro-ROS/XRCE-DDS '
                f'({_MICRO_ROS_MTU_BYTES} bytes). El agente los va a descartar en '
                f'silencio (mensaje "Trying to serialize N in {_MICRO_ROS_MTU_BYTES} '
                f'MTU stream") y el ESP32 nunca los va a recibir. Se ajusta a '
                f'{MAX_POINTS_PER_CHUNK_FOR_MTU} puntos/chunk.')
            points_per_chunk = MAX_POINTS_PER_CHUNK_FOR_MTU
        self._points_per_chunk = points_per_chunk

        chunk_publish_rate_hz = float(self.get_parameter('chunk_publish_rate_hz').value)
        self._max_joint_jump_rad = float(self.get_parameter('max_joint_jump_rad').value)
        self._command_duration_sec = float(self.get_parameter('command_duration_sec').value)
        arm_controller_topic = self.get_parameter('arm_controller_topic').value

        # Punto cartesiano de referencia para la siguiente interpolacion: arranca
        # en home y se actualiza a cada punto deseado nuevo que llega.
        self._last_target_point = np.array(home, dtype=np.float32)

        # Ultima posicion REAL conocida de cada articulacion (de /joint_states).
        # None hasta el primer mensaje: sin linea base real no se valida ningun
        # salto, asi que de entrada no se envia nada al arm_controller.
        self._current_joint_pos = {name: None for name in self._joint_names}

        self._chunks = []
        self._chunk_index = 0

        self._target_sub = self.create_subscription(
            Float32MultiArray, 'ik_test/target_point', self._target_callback, 10)
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_callback, 10)
        self._ik_result_sub = self.create_subscription(
            Float32MultiArray, 'ik_result_chunks', self._ik_result_callback,
            qos_profile_sensor_data)

        self._chunk_pub = self.create_publisher(
            Float32MultiArray, 'trajectory_chunks', qos_profile_sensor_data)
        self._trajectory_pub = self.create_publisher(
            JointTrajectory, arm_controller_topic, 10)
        self._jump_warning_pub = self.create_publisher(
            String, 'ik_test/joint_jump_warning', 10)

        self._chunk_timer = self.create_timer(
            1.0 / chunk_publish_rate_hz, self._chunk_timer_callback)

        self.get_logger().info(
            f'ik_bridge_node listo (home={home}, max_joint_jump_rad='
            f'{self._max_joint_jump_rad:.3f}). Esperando /joint_states y '
            f'/ik_test/target_point...')

    # ------------------------------------------------------------------
    # Punto deseado -> trayectoria cartesiana interpolada -> chunks al ESP
    # ------------------------------------------------------------------
    def _target_callback(self, msg):
        if len(msg.data) != POINT_STRIDE:
            self.get_logger().error(
                f'/ik_test/target_point debe tener {POINT_STRIDE} floats '
                f'[rx,ry,rz,nx,ny,nz,theta], recibidos {len(msg.data)}. Ignorado.')
            return

        start_point = self._current_cartesian_point()
        if start_point is None:
            self.get_logger().warn(
                'Aun no hay /joint_states completo para calcular la pose real '
                '(FK propia): se interpola desde el ultimo target comandado, '
                'que puede no coincidir con la pose real del brazo.',
                throttle_duration_sec=5.0)
            start_point = self._last_target_point

        target = np.array(msg.data, dtype=np.float32)
        path = self._interpolate(start_point, target, self._points_per_target)
        self._chunks = self._split_into_chunks(path, self._points_per_chunk)
        self._chunk_index = 0
        self._last_target_point = target

        self.get_logger().info(
            f'Punto deseado recibido: rx={target[0]:.4f} ry={target[1]:.4f} '
            f'rz={target[2]:.4f} theta={target[6]:.4f}. Interpolado en '
            f'{self._points_per_target} puntos / {len(self._chunks)} chunks.')

    def _current_cartesian_point(self):
        """Pose cartesiana REAL actual [rx,ry,rz,nx,ny,nz,theta], calculada
        con la FK propia (cuaternion dual) a partir de /joint_states. None
        si todavia no llegaron todas las articulaciones."""
        if any(self._current_joint_pos[name] is None for name in self._joint_names):
            return None

        th = [self._current_joint_pos[name] for name in self._joint_names]
        point, quat_norm = forward_kinematics_point(th)
        if abs(quat_norm - 1.0) > 1e-3:
            self.get_logger().warn(
                f'FK propia: norma de cuaternion = {quat_norm:.6f} (deberia '
                'ser 1.0). Revisar ik_bridge/forward_kinematics.py.',
                throttle_duration_sec=5.0)
        return point

    @staticmethod
    def _interpolate(start, end, num_points):
        # Interpolacion lineal de posicion y eje/angulo (con renormalizado del
        # eje). No es una SLERP real, pero alcanza para validar el lazo de IK
        # PC<->ESP32; no se usa para calidad de movimiento final.
        t = np.linspace(0.0, 1.0, num_points, dtype=np.float32).reshape(-1, 1)
        pos = start[0:3] + t * (end[0:3] - start[0:3])
        axis = start[3:6] + t * (end[3:6] - start[3:6])
        norms = np.linalg.norm(axis, axis=1, keepdims=True)
        norms[norms < 1e-6] = 1.0
        axis = axis / norms
        theta = (start[6] + t.flatten() * (end[6] - start[6])).reshape(-1, 1)
        return np.concatenate([pos, axis, theta], axis=1).astype(np.float32)

    @staticmethod
    def _split_into_chunks(points, chunk_size):
        return [points[start:start + chunk_size] for start in range(0, len(points), chunk_size)]

    def _chunk_timer_callback(self):
        if self._chunk_index >= len(self._chunks):
            return

        chunk = self._chunks[self._chunk_index]
        msg = Float32MultiArray()
        msg.data = chunk.flatten().tolist()
        self._chunk_pub.publish(msg)
        self._chunk_index += 1

        if self._chunk_index >= len(self._chunks):
            self.get_logger().info('Trayectoria hacia el punto deseado enviada por completo al ESP32.')

    # ------------------------------------------------------------------
    # Estado real del brazo, seguido en tiempo real
    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self._current_joint_pos:
                self._current_joint_pos[name] = position

    # ------------------------------------------------------------------
    # Resultado de IK del ESP32 -> chequeo de saltos -> arm_controller
    # ------------------------------------------------------------------
    def _ik_result_callback(self, msg):
        n = len(msg.data)
        if n % RESULT_STRIDE != 0:
            self.get_logger().warn(
                f'/ik_result_chunks con tamano {n} no es multiplo de {RESULT_STRIDE}. Ignorado.',
                throttle_duration_sec=5.0)
            return

        for i in range(0, n, RESULT_STRIDE):
            self._process_ik_point(msg.data[i:i + RESULT_STRIDE])

    @staticmethod
    def _angular_delta(target, current):
        # Distancia angular minima entre dos angulos, valida para juntas
        # 'continuous' cuyo /joint_states puede reportarse envuelto en
        # (-pi, pi]. Una resta cruda (abs(target - current)) da un salto
        # falso de ~2*pi cuando el angulo cruza la frontera +pi/-pi aunque
        # el movimiento real sea pequeno.
        raw = target - current
        wrapped = (raw + np.pi) % (2.0 * np.pi) - np.pi
        return abs(wrapped)

    def _process_ik_point(self, q):
        if any(self._current_joint_pos[name] is None for name in self._joint_names):
            self.get_logger().warn(
                'Aun no se recibio /joint_states: sin linea base real para '
                'validar saltos. Punto de IK descartado por seguridad.',
                throttle_duration_sec=5.0)
            return

        raw_deltas = {
            name: abs(q[i] - self._current_joint_pos[name])
            for i, name in enumerate(self._joint_names)
        }
        deltas = {
            name: self._angular_delta(q[i], self._current_joint_pos[name])
            for i, name in enumerate(self._joint_names)
        }
        jumped = {name: d for name, d in deltas.items() if d > self._max_joint_jump_rad}

        if jumped:
            detalle = ', '.join(
                f'{name}={d:.3f} rad (crudo={raw_deltas[name]:.3f} rad)'
                for name, d in jumped.items())

            current_point = self._current_cartesian_point()
            if current_point is not None:
                fk_txt = (
                    f' | FK real (efector final, base_link): '
                    f'x={current_point[0]:.4f} y={current_point[1]:.4f} '
                    f'z={current_point[2]:.4f} eje=({current_point[3]:.3f},'
                    f'{current_point[4]:.3f},{current_point[5]:.3f}) '
                    f'theta={current_point[6]:.4f} rad')
            else:
                fk_txt = ''

            warning_text = (
                f'Salto de articulacion detectado (> {self._max_joint_jump_rad:.3f} rad): '
                f'{detalle}. Punto de IK descartado, no se envia al arm_controller.'
                f'{fk_txt}')
            self.get_logger().warn(warning_text, throttle_duration_sec=1.0)

            warn_msg = String()
            warn_msg.data = warning_text
            self._jump_warning_pub.publish(warn_msg)
            return

        self._send_to_arm_controller(q)

    def _send_to_arm_controller(self, q):
        traj = JointTrajectory()
        traj.joint_names = self._joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        duration = DurationMsg()
        duration.sec = int(self._command_duration_sec)
        duration.nanosec = int((self._command_duration_sec - duration.sec) * 1e9)
        point.time_from_start = duration
        traj.points = [point]

        self._trajectory_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = IKBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
