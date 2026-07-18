import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

# Debe coincidir con MAX_POINTS_PER_CHUNK del firmware ESP32.
FIRMWARE_MAX_POINTS_PER_CHUNK = 100

# Formato de cada punto enviado al firmware: [rx, ry, rz, nx, ny, nz, theta]
POINT_STRIDE = 7


class TrajectoryPublisherNode(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')

        self.declare_parameter('radius', 0.05)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('z_height', 0.20)
        self.declare_parameter('num_points', 200)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('points_per_chunk', FIRMWARE_MAX_POINTS_PER_CHUNK)

        radius = self.get_parameter('radius').value
        center_x = self.get_parameter('center_x').value
        center_y = self.get_parameter('center_y').value
        z_height = self.get_parameter('z_height').value
        num_points = self.get_parameter('num_points').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        points_per_chunk = self.get_parameter('points_per_chunk').value

        if points_per_chunk > FIRMWARE_MAX_POINTS_PER_CHUNK:
            self.get_logger().warn(
                f'points_per_chunk={points_per_chunk} excede el limite del firmware '
                f'({FIRMWARE_MAX_POINTS_PER_CHUNK}). Se ajusta a {FIRMWARE_MAX_POINTS_PER_CHUNK}.'
            )
            points_per_chunk = FIRMWARE_MAX_POINTS_PER_CHUNK

        self._publisher = self.create_publisher(
            Float32MultiArray, '/trajectory_chunks', qos_profile_sensor_data
        )

        trajectory = self._generate_circle_trajectory(
            radius, center_x, center_y, z_height, num_points
        )
        self._chunks = self._split_into_chunks(trajectory, points_per_chunk)
        self._chunk_index = 0

        self.get_logger().info(
            f'Trayectoria circular generada: {num_points} puntos, radio={radius:.4f} m, '
            f'centro=({center_x:.4f}, {center_y:.4f}), z={z_height:.4f} m'
        )
        self.get_logger().info(
            f'Trayectoria dividida en {len(self._chunks)} chunks '
            f'(maximo {points_per_chunk} puntos/chunk, publicando a {publish_rate_hz:.2f} Hz)'
        )

        period_s = 1.0 / publish_rate_hz
        self._timer = self.create_timer(period_s, self._timer_callback)

    def _generate_circle_trajectory(self, radius, center_x, center_y, z_height, num_points):
        """Genera un circulo en el plano XY con orientacion fija (eje +Z, theta=0)."""
        angles = np.linspace(0.0, 2.0 * np.pi, num_points, endpoint=False, dtype=np.float32)

        rx = (center_x + radius * np.cos(angles)).astype(np.float32)
        ry = (center_y + radius * np.sin(angles)).astype(np.float32)
        rz = np.full(num_points, z_height, dtype=np.float32)

        # Eje de rotacion del efector fijo apuntando hacia arriba, sin rotacion adicional.
        nx = np.zeros(num_points, dtype=np.float32)
        ny = np.zeros(num_points, dtype=np.float32)
        nz = np.ones(num_points, dtype=np.float32)
        theta = np.zeros(num_points, dtype=np.float32)

        return np.stack([rx, ry, rz, nx, ny, nz, theta], axis=1).astype(np.float32)

    def _split_into_chunks(self, points, chunk_size):
        return [points[start:start + chunk_size] for start in range(0, len(points), chunk_size)]

    def _timer_callback(self):
        if self._chunk_index >= len(self._chunks):
            self.get_logger().info(
                'Trayectoria completa: todos los chunks fueron enviados. '
                'Nodo en espera (Ctrl+C para salir).'
            )
            self._timer.cancel()
            return

        chunk = self._chunks[self._chunk_index]
        msg = Float32MultiArray()
        msg.data = chunk.flatten().tolist()
        self._publisher.publish(msg)

        self.get_logger().info(
            f'Chunk {self._chunk_index + 1}/{len(self._chunks)} enviado: '
            f'{chunk.shape[0]} puntos ({len(msg.data)} floats, stride={POINT_STRIDE})'
        )
        self._chunk_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
