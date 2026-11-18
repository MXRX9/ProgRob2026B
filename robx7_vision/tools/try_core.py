import os
import sys

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from robx7_vision import detector_core as core  # noqa: E402

img = cv2.imread(sys.argv[1] if len(sys.argv) > 1 else '/tmp/frame.png')
if img is None:
    sys.exit('No se pudo leer la imagen')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
for color, ranges in core.COLOR_RANGES.items():
    cv2.imwrite(f'/tmp/mask_{color}.png', core.segment_color(hsv, ranges))

out = img.copy()
for b in core.find_blobs(img):
    shape = core.classify_shape(b.circularity)
    print(f'{b.color:6s} centroide=({b.cx:6.1f},{b.cy:6.1f}) area={b.area:6.0f} '
          f'circ={b.circularity:.2f} -> {shape}')
    cv2.drawContours(out, [b.contour], -1, (255, 255, 255), 2)
    cv2.putText(out, f'{b.color}/{shape}', (int(b.cx) - 30, int(b.cy) - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
cv2.imwrite('/tmp/blobs.png', out)
print('Escrito /tmp/blobs.png y /tmp/mask_*.png')
