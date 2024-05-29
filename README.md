### Como rodar o Projeto:

---

**Primeiramente é rocomendado que faça um Build no diretório principal:**

cd ponderada_turtlebot

source install/local_setup.bash

colcon build

---

**Depois disso agora inicie a simulação desejada, utilizei o Webots que pode ser inciado com seguinte comando:**

ros2 launch webots_ros2_turtlebot robot_launch.py

---

**Em um teminal rode o arquivo que inicia websocket, atraves do comando:**

cd src/turtlebot_control/turtlebot_control

python3 video_server.py

---

**Agora em outro terminal basta apenas rodar a interface, atraves do comando:**

ros2 run turtlebot_control turtlebot_controller

---

Agora deve estar tudo funcionando ai é só testar as funções e botões no webots e ver a camêra do seu notebook

LINK DO VIDEO FUNCIONANDO:

https://drive.google.com/file/d/1W2eDptg-TP4SXolvxRk6s6QqubPoJ5mw/view?usp=sharing
