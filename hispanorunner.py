import subprocess

comandos_por_terminal = [
    [
        "echo 'Terminal 1'",
        "cd $HOME/HispanoTech-UGC-ROS2",
        "colcon build",
        "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
        "ros2 launch hispanotech_map turtlebot3_hispano_world.launch.py"
    ],
    [
        "echo 'Terminal 2'",
        "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
        "ros2 launch provide_hispano_map provide_hispano_map.launch.py"
    ],
    [
        "echo 'Terminal 3'",
        "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
        "ros2 run turtlebot3_teleop teleop_keyboard"
    ],
    [
        "echo 'Terminal 4'",
        "source ~/HispanoTech-UGC-ROS2/install/setup.bash",
        'ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: $HOME//HispanoTech-UGC-ROS2/src/provide_hispano_map/map/my_map.yaml}"'
    ]
]

for i, comandos in enumerate(comandos_por_terminal):
    input(f"\n Pulsa ENTER para abrir la Terminal {i+1}...")
    comando_unico = "; ".join(comandos) + "; exec bash"
    subprocess.Popen(["gnome-terminal", "--", "bash", "-c", comando_unico])