# HispanoTech-UGC-ROS2

Repositorio oficial de exploración y navegación autónoma con ROS2 para TurtleBot3, desarrollado por el equipo HispanoTech dentro del marco de prácticas y proyectos académicos.

**Repositorio GitHub:**  
https://github.com/HispanoTech-UGC/HispanoTech-UGC-ROS2/

---

## Descripción general

Este proyecto se compone de múltiples paquetes ROS2 organizados para cubrir dos grandes funcionalidades:

- **SLAM en tiempo real:** mapeado del entorno con `slam_toolbox`.
- **Navegación/localización sobre mapa estático:** uso de `map_server` y mapas previamente generados.

---

## Estructura del repositorio

```
HispanoTech-UGC-ROS2/
├── src/
│   ├── hispano_slam/              # SLAM en tiempo real con slam_toolbox
│   │   ├── launch/
│   │   ├── config/
│   │   └── rviz/
│   ├── provide_hispano_map/       # Proveedor de mapas estáticos
│   │   ├── map/
│   │   ├── launch/
│   │   └── rviz/
│   ├── hispanotech_map/           # Mundo y entorno simulado en Gazebo
│   └── ...
├── hispanorunner.py               # Script Python para lanzar el sistema paso a paso
├── install/, build/, log/         # Generados tras compilar
```

---

## Paquetes incluidos

### 1. `hispano_slam`

Paquete para realizar SLAM en tiempo real con `slam_toolbox`.

- Lanza `slam_toolbox` en modo síncrono.
- Visualiza el entorno en RViz.
- Mapea mientras el robot se mueve.
- Parámetros personalizados en `config/slam_params.yaml`.

Lanzamiento:

```bash
ros2 launch hispano_slam slam_toolbox.launch.py
```

---

### 2. `provide_hispano_map`

Paquete para cargar un mapa ya existente mediante `map_server`.

- Carga `.yaml` y `.pgm` del mapa.
- Lanza RViz y visualiza el entorno.
- Usa `lifecycle_manager` y transformaciones necesarias.

Lanzamiento:

```bash
ros2 launch provide_hispano_map provide_hispano_map.launch.py
```

---

### 3. `hispanorunner.py`

Script Python auxiliar que automatiza el lanzamiento en orden de:

- Gazebo
- Mapa estático
- Teleoperación
- Carga y activación del mapa

Ejecución:

```bash
python3 hispanorunner.py
```

---

## Guardar mapas con SLAM

Tras completar la exploración:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/nombre_mapa
```

Esto genera:

- `nombre_mapa.yaml`
- `nombre_mapa.pgm`

Puedes moverlos al directorio `provide_hispano_map/map/` para su reutilización.

---

## Requisitos principales

```bash
sudo apt install   ros-humble-slam-toolbox   ros-humble-nav2-map-server   ros-humble-rviz2   ros-humble-turtlebot3*   ros-humble-tf2-tools
```

---

## Estado actual del proyecto

- [x] SLAM funcionando en tiempo real con RViz
- [x] Guardado y reutilización de mapas
- [x] Navegación con mapa estático cargado
- [x] Visualización completa con RViz
- [x] Automatización con `hispanorunner.py`

---

Este proyecto está en desarrollo activo dentro del equipo HispanoTech.
