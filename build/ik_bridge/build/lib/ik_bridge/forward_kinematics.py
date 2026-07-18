"""Cinematica directa propia (cuaternion dual) del brazo, evaluada sobre los
angulos reales de las articulaciones (/joint_states).

No depende del URDF/TF: el URDF exportado tiene un origen y una orientacion
de ejes de referencia distintos a los que se usaron para derivar la
cinematica inversa, asi que la pose que da robot_state_publisher no es
comparable con los puntos [rx,ry,rz,nx,ny,nz,theta] que espera el firmware.
Este modulo reimplementa, tal cual, las ecuaciones cerradas (posicion +
cuaternion de orientacion, parte real del cuaternion dual) provistas por el
usuario, en el orden de articulaciones:

    th0=base_link_revolucion-5, th1=arm1_revolucion-6, th2=arm2_revolucion-7,
    th3=arm3_revolucion-8, th4=arm4_revolucion-9, th5=arm5_revolucion-11

La posicion no depende de th5 (muneca terminal) y la orientacion depende de
las 6, lo cual es consistente con una muneca esferica en el ultimo eje.
"""

import numpy as np

# Terminos de w,x,y,z del cuaternion de orientacion (parte real del
# cuaternion dual), como suma de cos/sin de combinaciones de angulos/2.
# Cada tupla es (signo, 'cos'|'sin', (c0..c5)) donde el argumento del
# termino es c0*th0/2 + c1*th1/2 + ... + c5*th5/2, ci en {-1,+1}.
_W_TERMS = [
    (+1, 'cos', (-1, +1, +1, +1, +1, -1)),
    (-1, 'cos', (+1, -1, -1, -1, +1, +1)),
    (+1, 'cos', (+1, -1, -1, +1, -1, +1)),
    (+1, 'cos', (+1, -1, -1, +1, +1, +1)),
    (-1, 'cos', (+1, +1, +1, -1, -1, +1)),
    (+1, 'cos', (+1, +1, +1, -1, +1, +1)),
    (+1, 'cos', (+1, +1, +1, +1, -1, +1)),
    (+1, 'cos', (+1, +1, +1, +1, +1, +1)),
]

_X_TERMS = [
    (+1, 'cos', (+1, -1, -1, +1, +1, -1)),
    (-1, 'cos', (-1, +1, +1, +1, -1, +1)),
    (-1, 'cos', (-1, +1, +1, +1, +1, +1)),
    (-1, 'cos', (-1, +1, +1, -1, +1, +1)),
    (+1, 'cos', (+1, +1, +1, -1, -1, -1)),
    (+1, 'cos', (+1, +1, +1, -1, +1, -1)),
    (-1, 'cos', (+1, +1, +1, +1, -1, -1)),
    (+1, 'cos', (+1, +1, +1, +1, +1, -1)),
]

_Y_TERMS = [
    (+1, 'sin', (-1, +1, +1, -1, +1, +1)),
    (+1, 'sin', (-1, +1, +1, +1, -1, +1)),
    (+1, 'sin', (-1, +1, +1, +1, +1, +1)),
    (+1, 'sin', (+1, -1, -1, +1, +1, -1)),
    (+1, 'sin', (+1, +1, +1, -1, -1, -1)),
    (+1, 'sin', (+1, +1, +1, -1, +1, -1)),
    (-1, 'sin', (+1, +1, +1, +1, -1, -1)),
    (+1, 'sin', (+1, +1, +1, +1, +1, -1)),
]

_Z_TERMS = [
    (+1, 'sin', (+1, -1, -1, +1, -1, +1)),
    (-1, 'sin', (+1, -1, -1, -1, +1, +1)),
    (-1, 'sin', (-1, +1, +1, +1, +1, -1)),
    (+1, 'sin', (+1, -1, -1, +1, +1, +1)),
    (-1, 'sin', (+1, +1, +1, -1, -1, +1)),
    (+1, 'sin', (+1, +1, +1, -1, +1, +1)),
    (+1, 'sin', (+1, +1, +1, +1, -1, +1)),
    (+1, 'sin', (+1, +1, +1, +1, +1, +1)),
]


def _eval_terms(terms, half_angles):
    total = 0.0
    for sign, kind, coeffs in terms:
        arg = sum(c * a for c, a in zip(coeffs, half_angles))
        trig = np.cos(arg) if kind == 'cos' else np.sin(arg)
        total += sign * trig / 4.0
    return total


def fk_orientation_quaternion(th):
    """Cuaternion (w, x, y, z) de orientacion a partir de th0..th5 (rad)."""
    half_angles = [t / 2.0 for t in th]
    w = _eval_terms(_W_TERMS, half_angles)
    x = _eval_terms(_X_TERMS, half_angles)
    y = _eval_terms(_Y_TERMS, half_angles)
    z = _eval_terms(_Z_TERMS, half_angles)
    return np.array([w, x, y, z], dtype=np.float64)


def fk_position(th):
    """Posicion (x, y, z) a partir de th0..th4 (th5 no afecta la posicion)."""
    th0, th1, th2, th3, th4, _th5 = th
    c, s = np.cos, np.sin

    px = (
        (9 * c(th0) * s(th1)) / 100
        - (87 * s(th0) * s(th3) * s(th4)) / 1000
        + (131 * c(th0) * c(th1) * s(th2)) / 1000
        + (131 * c(th0) * c(th2) * s(th1)) / 1000
        + (87 * c(th0) * c(th1) * c(th4) * s(th2)) / 1000
        + (87 * c(th0) * c(th2) * c(th4) * s(th1)) / 1000
        + (87 * c(th0) * c(th1) * c(th2) * c(th3) * s(th4)) / 1000
        - (87 * c(th0) * c(th3) * s(th1) * s(th2) * s(th4)) / 1000
    )

    py = (
        (9 * s(th0) * s(th1)) / 100
        + (131 * c(th1) * s(th0) * s(th2)) / 1000
        + (131 * c(th2) * s(th0) * s(th1)) / 1000
        + (87 * c(th4 / 2) * s(th4 / 2) * c(th0) * s(th3)) / 500
        + (87 * c(th1) * c(th4) * s(th0) * s(th2)) / 1000
        + (87 * c(th2) * c(th4) * s(th0) * s(th1)) / 1000
        + (87 * c(th0 / 2) * c(th4 / 2) * s(th0 / 2) * s(th4 / 2) * c(th1) * c(th2) * c(th3)) / 250
        - (174 * c(th0 / 2) * c(th1 / 2) * c(th2 / 2) * c(th4 / 2) * s(th0 / 2) * s(th1 / 2) * s(th2 / 2) * s(th4 / 2) * c(th3)) / 125
    )

    pz = (
        (9 * c(th1)) / 100
        + (131 * c(th1) * c(th2)) / 1000
        - (131 * s(th1) * s(th2)) / 1000
        - (87 * c(th4) * s(th1) * s(th2)) / 1000
        + (87 * c(th1) * c(th2) * c(th4)) / 1000
        - (87 * c(th1) * c(th3) * s(th2) * s(th4)) / 1000
        - (87 * c(th2) * c(th3) * s(th1) * s(th4)) / 1000
        + 22 / 125
    )

    return np.array([px, py, pz], dtype=np.float64)


def quaternion_to_axis_angle(quat, eps=1e-9):
    """Convierte (w,x,y,z) -> (eje unitario, theta en [0, pi]).

    Normaliza primero (por si el cuaternion no viene perfectamente unitario
    por acumulacion de error numerico) y fuerza w >= 0 para quedarse con la
    representacion de angulo mas corto (q y -q son la misma rotacion).
    """
    w, x, y, z = quat
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    if norm < eps:
        return np.array([1.0, 0.0, 0.0]), 0.0

    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    if w < 0:
        w, x, y, z = -w, -x, -y, -z

    theta = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sqrt(max(1.0 - w * w, 0.0))
    if s < eps:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = np.array([x, y, z]) / s
    return axis, theta


def forward_kinematics_pose(th):
    """th0..th5 (rad) -> (posicion xyz, cuaternion wxyz, norma_cuaternion).

    La norma se devuelve para diagnostico: por construccion el cuaternion de
    orientacion deberia ser siempre unitario (~1.0); si se aleja de 1.0 hay
    un error de transcripcion en las ecuaciones de arriba.
    """
    pos = fk_position(th)
    quat = fk_orientation_quaternion(th)
    quat_norm = float(np.linalg.norm(quat))
    return pos, quat, quat_norm


def forward_kinematics_point(th):
    """th0..th5 (rad) -> ([rx,ry,rz,nx,ny,nz,theta], norma_cuaternion).

    Mismo calculo que forward_kinematics_pose pero con la orientacion ya
    convertida a eje-angulo, para encajar directo en el formato de punto
    [rx,ry,rz,nx,ny,nz,theta] usado por ik_bridge_node / el firmware.
    """
    pos, quat, quat_norm = forward_kinematics_pose(th)
    axis, theta = quaternion_to_axis_angle(quat)
    point = np.concatenate([pos, axis, [theta]]).astype(np.float32)
    return point, quat_norm
