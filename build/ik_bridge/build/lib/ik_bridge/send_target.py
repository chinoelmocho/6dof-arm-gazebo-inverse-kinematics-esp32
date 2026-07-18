"""Publica UN punto deseado en /ik_test/target_point y termina solo.

Pensado para reemplazar el "ros2 topic pub --once ... std_msgs/msg/..." a
mano: mas corto, valida la cantidad de numeros, espera a que ik_bridge_node
este realmente suscripto (si no, avisa y no se queda colgado para siempre),
y sale con codigo de error si no pudo entregarlo.

Uso:
    ros2 run ik_bridge send_target                       # usa el home_point
    ros2 run ik_bridge send_target 0.05 0.05 0.30 1 0 0 -1.53
"""

import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ik_bridge.ik_bridge_node import DEFAULT_HOME_POINT, POINT_STRIDE


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Publica un punto [rx ry rz nx ny nz theta] en '
                    '/ik_test/target_point.')
    parser.add_argument(
        'point', nargs='*', type=float,
        help=f'{POINT_STRIDE} numeros: rx ry rz nx ny nz theta. Si se '
             f'omiten, se usa el home_point por defecto ({DEFAULT_HOME_POINT}).')
    parser.add_argument('--topic', default='ik_test/target_point')
    parser.add_argument(
        '--timeout', type=float, default=5.0,
        help='segundos maximos a esperar que algo (ik_bridge_node) este '
             'suscripto antes de rendirse (default 5s).')
    return parser.parse_args(argv)


def main(args=None):
    ns = _parse_args(sys.argv[1:] if args is None else args)

    if ns.point:
        if len(ns.point) != POINT_STRIDE:
            print(
                f'Se esperaban {POINT_STRIDE} numeros [rx,ry,rz,nx,ny,nz,theta], '
                f'se recibieron {len(ns.point)}: {ns.point}',
                file=sys.stderr)
            sys.exit(1)
        point = ns.point
    else:
        point = list(DEFAULT_HOME_POINT)
        print(f'Sin argumentos: uso el home_point por defecto {point}')

    rclpy.init()
    node = Node('ik_bridge_send_target')
    pub = node.create_publisher(Float32MultiArray, ns.topic, 10)

    deadline = time.monotonic() + ns.timeout
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        node.get_logger().info(
            f'Esperando que algo se suscriba a /{ns.topic} '
            '(¿esta corriendo ik_bridge_node?)...',
            throttle_duration_sec=1.0)
        rclpy.spin_once(node, timeout_sec=0.2)

    if pub.get_subscription_count() == 0:
        node.get_logger().error(
            f'Nadie se suscribio a /{ns.topic} en {ns.timeout:.0f}s. '
            'Punto NO enviado (¿ik_bridge_node esta corriendo?).')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    msg = Float32MultiArray()
    msg.data = [float(v) for v in point]
    pub.publish(msg)
    node.get_logger().info(f'Punto publicado en /{ns.topic}: {point}')

    # Pequeno margen para que el mensaje termine de salir antes de cerrar.
    rclpy.spin_once(node, timeout_sec=0.3)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
