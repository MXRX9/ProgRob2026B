import cv2
import numpy as np

from robx7_vision import detector_core as core

W, H, HFOV = 640, 480, 1.2
FX = (W / 2) / np.tan(HFOV / 2)
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1.0]])


def rpy_to_matrix(r, p, y):
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


# Pose del marco optico en 'world'
R_WC = rpy_to_matrix(0, np.pi / 2, 0) @ rpy_to_matrix(-np.pi / 2, 0, -np.pi / 2)
T_WC = np.array([0.65, 0.0, 0.8])
TOP = 0.04   # altura de la cara superior


def test_rotation_of_optical_frame():
    expected = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]], float)
    assert np.allclose(R_WC, expected, atol=1e-9)
    s = np.sqrt(0.5)
    assert np.allclose(core.quat_to_matrix(s, -s, 0, 0), expected, atol=1e-9)


def test_projection_roundtrip_and_known_pixels():
    u, v = core.world_to_pixel([0.66, 0.08, TOP], K, R_WC, T_WC)
    assert abs(u - 271) < 1.5 and abs(v - 234) < 1.5          # valores de la guia de la sesion 1
    p = core.pixel_to_world(u, v, K, R_WC, T_WC, TOP)
    assert np.allclose(p, [0.66, 0.08, TOP], atol=1e-6)


def _scene():
    img = np.full((H, W, 3), (115, 158, 191), np.uint8)       # mesa (BGR)
    truth = {'red': (0.66, 0.08), 'green': (0.74, -0.07), 'blue': (0.64, -0.14)}
    bgr = {'red': (13, 13, 230), 'green': (13, 204, 13), 'blue': (230, 13, 13)}
    for color, (x, y) in truth.items():
        u, v = core.world_to_pixel([x, y, TOP], K, R_WC, T_WC)
        if color == 'green':                                   # cilindro (circulo visto de arriba)
            cv2.circle(img, (int(round(u)), int(round(v))), 12, bgr[color], -1)
        else:                                                  # cubo (cuadrado rotado)
            rect = ((u, v), (24, 24), 20.0)
            cv2.fillPoly(img, [np.intp(cv2.boxPoints(rect))], bgr[color])
    # Placa roja grande (destino) que debe descartarse por area
    cv2.rectangle(img, (0, 290), (70, 370), (13, 13, 150), -1)
    return img, truth


def test_detection_classification_and_pose():
    img, truth = _scene()
    blobs = core.find_blobs(img)
    assert sorted(b.color for b in blobs) == ['blue', 'green', 'red']   # la placa se descarto
    for b in blobs:
        p = core.pixel_to_world(b.cx, b.cy, K, R_WC, T_WC, TOP)
        x, y = truth[b.color]
        assert np.hypot(p[0] - x, p[1] - y) < 0.005, b.color   # < 5 mm
        shape = core.classify_shape(b.circularity)
        assert shape == ('cylinder' if b.color == 'green' else 'cube'), (b.color, b.circularity)


def test_intrinsics_from_fov():
    k = core.intrinsics_from_fov(640, 480, 1.2)
    assert np.allclose(k, K)
