import subprocess
import time

def lanzar_terminal(comandos):
    comando_unico = "; ".join(comandos) + "; exec bash"
    return subprocess.Popen(["gnome-terminal", "--", "bash", "-c", comando_unico])

def esperar_frame(frame, timeout=60):
    print(f"⏳ Esperando a que el frame '{frame}' esté disponible (timeout {timeout}s)...")
    start_time = time.time()
    while True:
        try:
            result = subprocess.run(
                ["ros2", "run", "tf2_ros", "tf2_echo", frame, frame],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5
            )
            if result.returncode == 0:
                print(f"✅ Frame '{frame}' disponible.")
                return True
        except subprocess.TimeoutExpired:
            pass

        if time.time() - start_time > timeout:
            print(f"❌ Timeout esperando el frame '{frame}'.")
            return False

        time.sleep(1)

# Terminal 1: Lanzar el mundo
lanzar_terminal([
    "echo 'Terminal 1 - Mundo'",
    "cd $HOME/HispanoTech-UGC-ROS2",
    "colcon build",
    "source install/setup.bash",
    "ros2 launch hispanotech_map turtlebot3_hispano_world.launch.py"
])

input("🔁 Espera a que el mundo cargue completamente y pulsa ENTER...")

# Terminal 2: Lanzar el nodo del mapa
lanzar_terminal([
    "echo 'Terminal 2 - Mapa'",
    "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
    "ros2 launch provide_hispano_map provide_hispano_map.launch.py"
])

input("🔁 Espera a que el nodo del mapa esté lanzado y pulsa ENTER...")

# Terminal 3: Teleoperación
lanzar_terminal([
    "echo 'Terminal 3 - Teleop'",
    "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
    "ros2 run turtlebot3_teleop teleop_keyboard"
])

# Esperar a que el frame 'map' esté disponible (tras activarlo)
# Esto es para más adelante, después del siguiente paso

# Terminal 4: Llamada al servicio de cargar mapa
lanzar_terminal([
    "echo 'Terminal 4 - Cargar mapa'",
    "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
    'ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: $HOME/HispanoTech-UGC-ROS2/src/hispanotech_nav_system/config/hispanotech_map.yaml}"'
])

input("🔁 Pulsa ENTER cuando se haya cargado el mapa para activar el map_server...")
# Terminal 5: Activar map_server si no lo está ya
print("🔍 Comprobando si map_server ya está activo...")

estado = subprocess.check_output([
    "ros2", "lifecycle", "get", "/map_server"
], text=True)

if "active" in estado.lower():
    print("✅ map_server ya está activo.")
else:
    print("🟡 map_server no está activo, activándolo ahora...")
    lanzar_terminal([
        "echo 'Activando map_server'",
        "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
        "ros2 lifecycle set /map_server activate"
    ])

# Esperar a que el frame 'map' esté disponible tras activación
esperar_frame("map")
