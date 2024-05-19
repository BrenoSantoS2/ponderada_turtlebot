#Código feito para executar as principais comandos de maneira automática e fácil para que a aplicação funcione
#Facilitando a vida do usuário 
trap "echo 'Limpando... (apagando venv)'; rm -rf $(pwd)/src/venv" EXIT

cd src

python3 -m venv venv

source venv/bin/activate

pip install typer inquirer

cd project_ws

TYPER_PATH=$(pip show typer | grep "Location:" | awk '{print $2}')

if [ -z "$TYPER_PATH" ]; then
    echo "Typer not found. Please make sure it is installed."
    exit 1
fi

export PYTHONPATH="$PYTHONPATH:$TYPER_PATH"


echo "Updated PYTHONPATH: $PYTHONPATH"

echo 'export ROS_DOMAIN_ID=86 #TURTLEBOT3' >> ~/.bashrc

colcon build

source install/local_setup.bash

ros2 run turtlebot_control control