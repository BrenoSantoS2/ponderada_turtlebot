import typer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import inquirer
import threading
import sys, os
import time

#Importa caso seja Windows
if os.name == 'nt':
    import msvcrt

#Importar caso seja Unix    
else:
    import tty, termios

app = typer.Typer()

# Cria um nó responsável por publicar mensagem no tópico cmd_vel e armazes algumas informações como velocidade linear, angular...
class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.is_connected = False
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        self.emergency_stop = False

    # Responsável por alterar o status do robo para Conectado.
    def connect(self):
        if not self.is_connected:
            self.is_connected = True
            print("Robô conectado e pronto para publicação.")

    #Responsável por alterar o status do robo para Desconectado.
    def disconnect(self):
        if self.is_connected:
            self.is_connected = False
            print("Robô desconectado. Para utilizá-lo, conecte-o novamente.")

    #Altera a mensagem que esta sendo enviada para o tópico onde o robo esta recebendo.
    def update_velocity(self):
        if self.is_connected:
            msg = Twist()
            msg.linear.x = self.lin_speed
            msg.angular.z = self.ang_speed
            self.publisher_.publish(msg)

    #Zera a velocidade linear e angular obrigando o robo a parar.
    def stop(self):
        self.lin_speed = 0.0
        self.ang_speed = 0.0
        self.update_velocity()
        print("Parando robô.")

    #Aumenta gradualmente a velocidade linear do robo.
    def increase_lin_speed(self):
        self.lin_speed += 0.1
        self.log_speeds()

    #Diminui gradualmente a velocidade linear do robo.
    def decrease_lin_speed(self):
        self.lin_speed -= 0.1
        self.log_speeds()

    #Aumenta gradualmente a velocidade angular do robo.
    def increase_ang_speed(self):
        self.ang_speed += 0.1
        self.log_speeds()

    #Diminui gradualmente a velocidade angular do robo.
    def decrease_ang_speed(self):
        self.ang_speed -= 0.1
        self.log_speeds()

    #Monstra a velocidade linear e angular atual para o usuário.
    def log_speeds(self):
        print(f"Velocidade linear: {self.lin_speed:.2f} m/s | Velocidade Angular: {self.ang_speed:.2f} rad/s")

    #Define que o robo pode ser operado (Pois não há uma chamada de parada de emergência).
    def start(self):
        self.emergency_stop = False

    #Checa se existe alguma chamada de parada de emergência.
    def check_emergency(self):
        return self.emergency_stop

    #Define que o robo não pode ser operado e trava completamente seus comandos (Pois há uma chamada de parada de emergência).
    def emergency_shutdown(self):
        self.emergency_stop = True
        self.stop()
        print("Parada de emergência ativada.")

#Função usada para reconhecer a entrada de teclas para movimentação do robo.
def get_key(settings):

    #Caso seja um Sistema Windows.
    if os.name == 'nt':
        if msvcrt.kbhit():
            return msvcrt.getch().decode()
        return ''
    
    #Caso seja um Sistema baseados em Unix.
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


#Função usada para fazer a teleoperação do TurtleBot.
def teleop_mode(controller):
    settings = None
 
    try:
        #Pega as configurações do terminal para sistemas baseados em Unix.
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        #Print dos comandos para que o usuário saiba como controlar.
        print("Entrando no modo de teleoperação. Use as seguintes teclas para controlar o robô:")
        print(" Use 'w', 's', 'a', 'd' para mover.")
        print(" Use 'espaço' para parar.")
        print(" Use 'q' para sair.")
        print(" Use 'b' para forçar a parada de emergência.")
        
        #Checa qual tecla foi apertada e faz com que uma função seja atribuida a ela.
        while True:
            key = get_key(settings)
            if key == 'w':
                controller.increase_lin_speed()
            elif key == 's':
                controller.decrease_lin_speed()
            elif key == 'a':
                controller.increase_ang_speed()
            elif key == 'd':
                controller.decrease_ang_speed()
            elif key == ' ':
                controller.stop()
            elif key == 'b':
                controller.emergency_shutdown()
                break
            elif key == 'q':
                break
            controller.update_velocity()
            time.sleep(0.1)

    finally:
        #Restaura as configurações iniciais dos terminais do sistema operacional baseados em Unix.
        if os.name != 'nt' and settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#Faz com que o robo sempre receba uma mensagem de movimento com isso você altera apenas a velocidade angular e velocidade linear.
#Dessa forma o robo continua seguindo o comando previamente estipulado sem precisar que sempre esteja precionando um tecla.
def continuous_update(controller):
    while rclpy.ok() and not controller.emergency_stop:
        controller.update_velocity()
        time.sleep(0.1)

#CLI responsável por concentrar toda a interação do usuáio com o robo.
def user_interface(controller):

    #Perguntas principais.
    main_menu = [
        inquirer.List('action',
                      message="Qual ação você quer realizar? (Conecte o robô antes de teleoperar)",
                      choices=['Teleoperar', 'Conectar', 'Desconectar', "Parada de emergência", 'Sair'])
    ]
    #Perguntas caso tenha um chamado de parada de emergência
    emergency_menu = [
        inquirer.List('action', message="Ação necessária:", choices=['Iniciar processo', 'Sair'])
    ]
    
    #Checa paradas e emergência e atribuiações dependendo a da função escolhida pelo usuário.
    while True:

        #Funções das perguntas principais
        if not controller.check_emergency():
            answers = inquirer.prompt(main_menu)
            action = answers['action']
            if action == 'Teleoperar':
                teleop_mode(controller)
            elif action == 'Conectar':
                controller.connect()
            elif action == 'Desconectar':
                controller.disconnect()
            elif action == 'Parada de emergência':
                controller.emergency_shutdown()
            elif action == 'Sair':
                break

        #Funções caso tenha um chamado de parada de emergência
        else:
            answers = inquirer.prompt(emergency_menu)
            action = answers['action']
            if action == "Iniciar processo":
                controller.start()
            elif action == 'Sair':
                break
    
    #Desconecta a conexão com o robo.
    controller.disconnect()
    rclpy.shutdown()
    exit(1)

#Inicia uma função Typer
@app.command()
def main():

    #Incia o ROS2
    rclpy.init()
    turtlebot_controller = TurtleBotController()
    
    #Faz com que o robo execute o comando de movimento constantemente em paralelo com outras funções.
    update_thread = threading.Thread(target=continuous_update, args=(turtlebot_controller,))
    update_thread.start()
    
    #Faz com que o controle do usuário funcione paralelamente com o robo.
    user_thread = threading.Thread(target=user_interface, args=(turtlebot_controller,))
    user_thread.start()

    #roda a aplicação
    try:
        rclpy.spin(turtlebot_controller)
        user_thread.join()
        update_thread.join()
    except KeyboardInterrupt:
        turtlebot_controller.disconnect()
        rclpy.shutdown()
        user_thread.join()
        update_thread.join()

#Executa a aplicação caso seja o arquivo principal
if __name__ == '__main__':
    app()

