"""Nodo standalone que expone la cinematica directa propia (forward_kinematics.py)
como algo visible y verificable en tiempo real: no depende del URDF/TF (que
tiene un origen/orientacion distinto al de tu cinematica), sino que evalua
tus ecuaciones cerradas directamente sobre /joint_states.

Publica:
  - ik_bridge/fk_pose (geometry_msgs/PoseStamped): posicion + orientacion
    (cuaternion) del efector final, en el frame 'base_link'.
  - Log periodico con la misma info en x,y,z + eje-angulo + cuaternion, para
    comparar a mano contra tu propia cinematica directa/inversa.
  - (opcional) mueve una esferita marcador dentro de Gazebo Classic, en la
    posicion real del efector final, para verificar visualmente si el brazo
    llega al punto que dice el log.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from ik_bridge.forward_kinematics import forward_kinematics_pose, quaternion_to_axis_angle
from ik_bridge.ik_bridge_node import DEFAULT_JOINT_NAMES

try:
    from gazebo_msgs.srv import SpawnEntity
    from gazebo_msgs.msg import ModelState
    _HAVE_GAZEBO_MSGS = True
except ImportError:
    _HAVE_GAZEBO_MSGS = False

_MARKER_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="fk_marker">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere><radius>0.015</radius></sphere>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
          <emissive>0 0.8 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


class FKNode(Node):
    """Sigue /joint_states y calcula, con la FK propia, donde esta realmente
    el efector final (posicion + orientacion), independiente del URDF."""

    def __init__(self):
        super().__init__('fk_node')

        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('spawn_marker_in_gazebo', True)
        self.declare_parameter('gazebo_model_name', 'BRAZOROBOTICO')

        self._joint_names = list(self.get_parameter('joint_names').value)
        self._current_joint_pos = {name: None for name in self._joint_names}
        self._spawn_marker = bool(self.get_parameter('spawn_marker_in_gazebo').value)
        self._gazebo_model_name = self.get_parameter('gazebo_model_name').value
        log_period = float(self.get_parameter('log_period_sec').value)

        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_callback, 10)
        self._pose_pub = self.create_publisher(PoseStamped, 'ik_bridge/fk_pose', 10)

        self._last_pose = None  # (pos, quat) del ultimo calculo valido
        self._marker_ready = False
        self._spawn_client = None
        self._model_state_pub = None

        if self._spawn_marker:
            if not _HAVE_GAZEBO_MSGS:
                self.get_logger().warn(
                    'gazebo_msgs no esta disponible: no se puede spawnear el '
                    'marcador visual en Gazebo. Se sigue publicando '
                    'ik_bridge/fk_pose y el log igual.')
            else:
                self._spawn_client = self.create_client(SpawnEntity, '/gazebo/spawn_entity')
                self._model_state_pub = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)
                self._spawn_marker_entity()

        self._log_timer = self.create_timer(log_period, self._log_pose)

        self.get_logger().info(
            'fk_node listo (FK propia, cuaternion dual, independiente del '
            'URDF). Esperando /joint_states...')

    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self._current_joint_pos:
                self._current_joint_pos[name] = position
        self._update_pose()

    def _update_pose(self):
        if any(v is None for v in self._current_joint_pos.values()):
            return

        th = [self._current_joint_pos[name] for name in self._joint_names]
        pos, quat, quat_norm = forward_kinematics_pose(th)
        if abs(quat_norm - 1.0) > 1e-3:
            self.get_logger().warn(
                f'fk_node: norma de cuaternion = {quat_norm:.6f} (deberia '
                'ser 1.0). Revisar ik_bridge/forward_kinematics.py.',
                throttle_duration_sec=5.0)

        self._last_pose = (pos, quat)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = float(pos[0])
        pose_msg.pose.position.y = float(pos[1])
        pose_msg.pose.position.z = float(pos[2])
        pose_msg.pose.orientation.w = float(quat[0])
        pose_msg.pose.orientation.x = float(quat[1])
        pose_msg.pose.orientation.y = float(quat[2])
        pose_msg.pose.orientation.z = float(quat[3])
        self._pose_pub.publish(pose_msg)

        if self._marker_ready:
            state = ModelState()
            state.model_name = 'fk_marker'
            state.pose = pose_msg.pose
            state.reference_frame = f'{self._gazebo_model_name}::base_link'
            self._model_state_pub.publish(state)

    # ------------------------------------------------------------------
    # Marcador visual en Gazebo Classic (esfera verde en el efector final
    # calculado por la FK propia, relativa a base_link).
    # ------------------------------------------------------------------
    def _spawn_marker_entity(self):
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                '/gazebo/spawn_entity no disponible: no se pudo spawnear el '
                'marcador visual (¿Gazebo no esta corriendo todavia?). '
                'Reinicia fk_node cuando Gazebo este listo si lo necesitas.')
            return

        req = SpawnEntity.Request()
        req.name = 'fk_marker'
        req.xml = _MARKER_SDF
        req.reference_frame = f'{self._gazebo_model_name}::base_link'
        future = self._spawn_client.call_async(req)
        future.add_done_callback(self._on_marker_spawned)

    def _on_marker_spawned(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Fallo al spawnear el marcador FK: {exc}')
            return

        if response.success:
            self._marker_ready = True
            self.get_logger().info(
                'Marcador FK (esfera verde) spawneado en Gazebo, siguiendo '
                'el efector final segun la FK propia.')
        else:
            self.get_logger().error(
                f'Gazebo rechazo el spawn del marcador FK: {response.status_message}')

    # ------------------------------------------------------------------
    def _log_pose(self):
        if self._last_pose is None:
            return
        pos, quat = self._last_pose
        axis, theta = quaternion_to_axis_angle(quat)
        self.get_logger().info(
            f'FK real (base_link): x={pos[0]:.4f} y={pos[1]:.4f} z={pos[2]:.4f} '
            f'| eje=({axis[0]:.3f},{axis[1]:.3f},{axis[2]:.3f}) theta={theta:.4f} rad '
            f'| quat=(w={quat[0]:.4f}, x={quat[1]:.4f}, y={quat[2]:.4f}, z={quat[3]:.4f})')


def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
