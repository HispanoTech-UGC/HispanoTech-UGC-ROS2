import subprocess
import time

# Códigos ANSI para colores
RESET = "\033[0m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"

# Configuración fija
IP_ROBOT = "192.168.0.95"
PASSWORD_ROBOT = "turtlebot"

# @function lanzar_terminal
# @brief Lanza una terminal con los comandos especificados
# @param comandos: Lista de comandos a ejecutar en la terminal
def lanzar_terminal(comandos):
    comando_unico = "; ".join(comandos) + "; exec bash"
    print(f"{YELLOW}[INFO]{RESET} Lanzando terminal con: {comando_unico}")
    try:
        return subprocess.Popen(["gnome-terminal", "--", "bash", "-c", comando_unico])
    except Exception as e:
        print(f"{RED}[ERROR]{RESET} No se pudo lanzar la terminal: {e}")

# @function ping_robot
# @brief Realiza un ping al robot para verificar su conectividad
# @param ip_robot: IP del robot
def ping_robot(ip_robot):
    print(f"{BLUE}Esperando a que el robot responda al ping...{RESET}")
    while True:
        try:
            subprocess.check_output(["ping", "-c", "1", ip_robot], stderr=subprocess.DEVNULL)
            print(f"{GREEN}[OK]{RESET} El robot con IP {ip_robot} está activo.")
            break
        except subprocess.CalledProcessError:
            print(f"{RED}[FAIL]{RESET} El robot no responde, esperando 5 segundos...")
            time.sleep(5)

# @function abrir_terminal_ssh_y_lanzar
# @brief Abre una terminal SSH interactiva que lanza un comando remoto tras conectarse
# @param ip_robot: IP del robot
# @param password: Contraseña del robot
# @param comando_remoto: Comando a lanzar en el robot tras el login
def abrir_terminal_ssh_y_lanzar(ip_robot, password, comando_remoto):
    print(f"{BLUE}Abriendo terminal SSH que lanzará el comando en el entorno del robot...{RESET}")
    
    comando_ssh = (
        f"sshpass -p {password} ssh -t -o StrictHostKeyChecking=no ubuntu@{ip_robot} "
        f"'bash --login -i -c \"{comando_remoto}\"'"
    )
    
    return lanzar_terminal([comando_ssh])
# @function lanzar_nav_system_local
# @brief Lanza el sistema de navegación hispano_nav_system en el entorno local

def lanzar_system_local(comando):
    print(f"{BLUE}Lanzando hispano_nav_system en tu PC...{RESET}")

    return lanzar_terminal([comando])

# MAIN
if __name__ == "__main__":
    print(f"{BLUE}Configuración del entorno de HispanoTech UGC ROS2{RESET}")

    ping_robot(IP_ROBOT)

    input(f"{YELLOW}Pulsa ENTER cuando quieras lanzar el bringup automáticamente...{RESET}")
    
    comando = "ros2 launch turtlebot3_bringup robot.launch.py"
    abrir_terminal_ssh_y_lanzar(IP_ROBOT, PASSWORD_ROBOT, comando)

    comando = "source install/setup.bash && ros2 launch hispano_nav_system hispano_nav_system.launch.py"
    lanzar_system_local(comando)

    comando = "cd ../HispanoTech-UGC-WEB && python3 -m http.server 8000"
    lanzar_system_local(comando)
    
    comando = "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    abrir_terminal_ssh_y_lanzar(IP_ROBOT, PASSWORD_ROBOT, comando)

    comando = "ros2 run image_tools cam2image --ros-args -p burger_mode:=false -p frequency:=10.0 -p reliability:=best_effort"
    abrir_terminal_ssh_y_lanzar(IP_ROBOT, PASSWORD_ROBOT, comando)

    print(f"{GREEN}Entorno de trabajo preparado. Puedes continuar.{RESET}")
