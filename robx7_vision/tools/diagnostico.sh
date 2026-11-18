source /opt/ros/humble/setup.bash 2>/dev/null
[ -f install/setup.bash ] && source install/setup.bash

ok()   { echo "  [OK]    $1"; }
fail() { echo "  [FALLO] $1"; echo "          -> $2"; }

echo "1) Camara en Gazebo (ign topic)"
if timeout 5 ign topic -l 2>/dev/null | grep -q "^/camera$"; then ok "/camera existe en Gazebo"
else fail "no existe /camera en Gazebo" "falta el sensor o el plugin Sensors del mundo (sesion 1, bloque 3)"; fi
if timeout 5 ign topic -l 2>/dev/null | grep -q "camera_info"; then ok "camera_info existe en Gazebo: $(timeout 5 ign topic -l | grep camera_info | tr '\n' ' ')"
else fail "no hay camera_info en Gazebo" "revisa el nombre real con: ign topic -l | grep camera"; fi

echo "2) Imagen en ROS 2"
if timeout 8 ros2 topic echo --once /camera/image_raw --field header >/tmp/diag_hdr.txt 2>&1; then
  ok "/camera/image_raw llega. $(grep frame_id /tmp/diag_hdr.txt | head -1)"
else fail "no llega /camera/image_raw" "revisa el nodo camera_bridge del launch"; fi

echo "3) camera_info en ROS 2"
if timeout 8 ros2 topic echo --once /camera/camera_info --field k >/tmp/diag_k.txt 2>&1; then
  ok "/camera/camera_info llega. K = $(tr -d '\n' </tmp/diag_k.txt | cut -c1-90)"
else fail "no llega /camera/camera_info" "es la causa mas comun de imagen de depuracion vacia. Usa 'ign topic -l | grep camera' y ajusta el puente; mientras tanto: -p fallback_hfov:=1.2"; fi

echo "4) TF: world -> camera_optical_link"
if timeout 8 ros2 run tf2_ros tf2_echo world camera_optical_link 2>&1 | grep -q "Translation"; then
  ok "la transformada existe"
else fail "no existe la transformada" "revisa robot_state_publisher y que robx7_camera.xacro este incluido"; fi

echo "5) Nodo de percepcion"
if ros2 node list 2>/dev/null | grep -q object_detector; then ok "object_detector esta corriendo"
else fail "object_detector NO esta corriendo" "mira la salida del launch: puede haber fallado al iniciar (import, interfaces sin compilar)"; fi
if timeout 8 ros2 topic echo --once /vision/image_debug --field header >/dev/null 2>&1; then ok "/vision/image_debug publica"
else fail "/vision/image_debug no publica" "el nodo no recibe imagenes: repite los pasos 2 y 3"; fi
if timeout 8 ros2 topic echo --once /vision/objects --field objects >/tmp/diag_obj.txt 2>&1 && grep -q "id:" /tmp/diag_obj.txt; then
  ok "hay objetos detectados: $(grep -o "id: [a-z_0-9']*" /tmp/diag_obj.txt | tr '\n' ' ')"
else fail "no hay objetos detectados" "abre /vision/image_debug: si dice SIN TF o ESPERANDO camera_info, ahi esta la causa; si no, ajusta los umbrales HSV (tools/try_core.py)"; fi
echo "Listo."
