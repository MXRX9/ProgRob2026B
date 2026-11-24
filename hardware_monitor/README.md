# hardware_monitor — Actividad 3 (Programación de Robots)

Sistema modular en ROS 2 Humble (Python) con tres nodos:

| Nodo | Ejecutable | Rol | Interfaz |
|---|---|---|---|
| `hardware_emulator` | `hardware_emulator` | Publisher (2 Hz) | `/hardware_status` (`robx7_interfaces/msg/HardwareStatus`) |
| `monitor_node` | `monitor_node` | Subscriber + Service Server | sub `/hardware_status`, srv `/check_system` |
| `operator_node` | `operator_node` | Service Client (cada 3 s) | cliente `/check_system` (`robx7_interfaces/srv/CheckSystem`) |

Parámetros:

- `monitor_node/umbral_temperatura` (int, por defecto **50**, solo lectura). El launch lo sobrescribe a **75**.
- `hardware_emulator/frecuencia_hz` (2.0) y `operator_node/periodo_consulta` (3.0).

## 1. Preparar el repositorio

1. En GitHub, haz un **Fork** de `angelparedes-lab/ProgRob2026B` (o crea un repo nuevo y sube el contenido del repo base).
2. Clona tu fork como carpeta `src` de un workspace:

```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/<TU_USUARIO>/ProgRob2026B.git src
```

3. Copia dentro de `~/ros2_ws/src/`:
   - la carpeta `hardware_monitor/` completa (paquete nuevo)
   - `robx7_interfaces/srv/CheckSystem.srv` (archivo nuevo)
   - `robx7_interfaces/CMakeLists.txt` (agrega la línea `"srv/CheckSystem.srv"`)

## 2. Compilar

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robx7_interfaces hardware_monitor
source install/setup.bash
```

## 3. Ejecutar (evidencia 1 y 2)

```bash
ros2 launch hardware_monitor hardware_system.launch.py
```

En esa terminal se ven los tres nodos. Toma la captura cuando aparezcan líneas
`[operator_node]` con `[OK] Autorización de movimiento concedida` y también
con `[BLOQUEO] Bloqueo por temperatura alta` (el ciclo completo tarda unos 30 s).

## 4. Exploración con la CLI (en otra terminal, haciendo `source` primero)

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

ros2 topic echo /hardware_status                      # publicación continua
ros2 param list                                       # parámetros de los nodos
ros2 param get /monitor_node umbral_temperatura       # debe decir: Integer value is: 75  (evidencia 4)
rqt_graph                                             # grafo de nodos (evidencia 3)
```

Opcional: `ros2 service call /check_system robx7_interfaces/srv/CheckSystem "{}"` llama al servicio a mano.
`ros2 param set /monitor_node umbral_temperatura 90` es rechazado porque el parámetro es de solo lectura (estático).

**rqt_graph:** los servicios no aparecen como flechas; elige "Nodes/Topics (all)" y desmarca
"Hide: Dead sinks / Leaf topics" para ver `hardware_emulator -> /hardware_status -> monitor_node`.
`operator_node` sí aparece como nodo. Para la evidencia del servicio sirve la captura de
`ros2 service list -t` y de `ros2 node info /monitor_node`.

## 5. Commit y tag de entrega

```bash
cd ~/ros2_ws/src
git add hardware_monitor robx7_interfaces
git commit -m "Actividad 3: sistema ROS 2 con topics, servicio, parámetros y launch"
git tag actividad_3/ros2-system
git push origin main
git push origin actividad_3/ros2-system
```

(Si tu rama se llama `master`, cambia `main` por `master`.)
