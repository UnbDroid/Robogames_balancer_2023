sudo dd if=/dev/sdb of=/home/thamires/Imagens/beagle/beaglebackup.img status=progress

sudo dd if=/home/thamires/Imagens/beagle/beaglebackup.img of=/dev/sdb status=progress


#!/bin/bash

cd /home/debian/beaglebone
./pin_off
echo "$(date) - Executando roscore" >> /var/log/meu_script.log
roscore
sleep 30
echo "$(date) - Fim execucao roscore" >> /var/log/meu_script.log
echo "$(date) - Executando imu" >> /var/log/meu_script.log
rosrun imu imu_node
sleep 2
echo "$(date) - Fim execucao imu" >> /var/log/meu_script.log
echo "$(date) - Executando equilibrio" >> /var/log/meu_script.log
rosrun equilibrio equilibrio_node
sleep 2
echo "$(date) - Fim execucao equilibrio" >> /var/log/meu_script.log
echo "$(date) - Executando velocidade" >> /var/log/meu_script.log
rosrun beginner_tutorials beginner_tutorials_node_listener
echo "$(date) - Fim executando velocidade" >> /var/log/meu_script.log

#!/bin/bash

cd /home/debian/beaglebone
./pin_off
echo "$(date) - Executando roscore" >> /var/log/meu_script.log
gnome-terminal -- bash -c "roscore; exec bash"
sleep 30
echo "$(date) - Fim execucao roscore" >> /var/log/meu_script.log
echo "$(date) - Executando imu" >> /var/log/meu_script.log
gnome-terminal -- bash -c "rosrun imu imu_node; exec bash"
sleep 1
echo "$(date) - Fim execucao imu" >> /var/log/meu_script.log
echo "$(date) - Executando equilibrio" >> /var/log/meu_script.log
gnome-terminal -- bash -c "rosrun equilibrio equilibrio_node; exec bash"
sleep 1
echo "$(date) - Fim execucao equilibrio" >> /var/log/meu_script.log
echo "$(date) - Executando velocidade" >> /var/log/meu_script.log
gnome-terminal -- bash -c "rosrun beginner_tutorials beginner_tutorials_node_listener; exec bash"
echo "$(date) - Fim executando velocidade" >> /var/log/meu_script.log


#!/bin/bash

sleep 100
screen -d -m -S "ros"
echo "$(date) - Executando roscore" >> /var/log/meu_script.log
sleep 60
screen -S "ros" -X screen bash -c 'roscore; exec bash'
echo "$(date) - Fim execucao roscore" >> /var/log/meu_script.log
sleep 60

screen -d -m -S "imu"
sleep 60
echo "$(date) - Executando imu" >> /var/log/meu_script.log
screen -S "imu" -X screen bash -c 'rosrun imu imu_node; exec bash'
echo "$(date) - Fim execucao imu" >> /var/log/meu_script.log

sleep 60
screen -d -m -S "equilibrio"
sleep 60
echo "$(date) - Executando equilibrio" >> /var/log/meu_script.log
screen -S "equilibrio" -X screen bash -c 'rosrun equilibrio equilibrio_node; exec bash'
echo "$(date) - Fim execucao equilibrio" >> /var/log/meu_script.log

sleep 60
screen -d -m -S "velocidade"
sleep 60
echo "$(date) - Executando velocidade" >> /var/log/meu_script.log
screen -S "velocidade" -X screen bash -c 'rosrun beginner_tutorials beginner_tutorials_node_listener; exec bash'
echo "$(date) - Fim executando velocidade" >> /var/log/meu_script.log


#!/bin/bash

echo "$(date) - Executando roscore" >> /var/log/meu_script.log
screen -dmS roscore bash -c 'roscore; exec bash'
echo "$(date) - Fim execucao roscore" >> /var/log/meu_script.log
sleep 5
echo "$(date) - Executando imu" >> /var/log/meu_script.log
screen -dmS imu bash -c 'rosrun imu imu_node; exec bash'
echo "$(date) - Fim execucao imu" >> /var/log/meu_script.log

echo "$(date) - Executando equilibrio" >> /var/log/meu_script.log
screen -dmS equilibrio bash -c 'rosrun equilibrio equilibrio_node; exec bash'
echo "$(date) - Fim execucao equilibrio" >> /var/log/meu_script.log

echo "$(date) - Executando velocidade" >> /var/log/meu_script.log
screen -dmS velocidade bash -c 'rosrun beginner_tutorials beginner_tutorials_node_listener; exec bash'
echo "$(date) - Fim executando velocidade" >> /var/log/meu_script.log

#!/bin/bash

screen -d -m -S ros
echo "$(date) - Executando roscore" >> /home/debian/meu_script.log
sleep 45
screen -S ros -X screen "roscore; exec bash"
echo "$(date) - Fim execucao roscore" >> /home/debian/meu_script.log
sleep 60

screen -d -m -S imu
sleep 45
echo "$(date) - Executando imu" >>  /home/debian/meu_script.log
screen -S imu -X screen "rosrun imu imu_node; exec bash"
echo "$(date) - Fim execucao imu" >>  /home/debian/meu_script.log

sleep 60
screen -d -m -S equilibrio
sleep 45
echo "$(date) - Executando equilibrio" >>  /home/debian/meu_script.log
screen -S equilibrio -X screen "rosrun equilibrio equilibrio_node; exec bash"
echo "$(date) - Fim execucao equilibrio" >>  /home/debian/meu_script.log

sleep 60
screen -d -m -S velocidade
sleep 45
echo "$(date) - Executando velocidade" >> /home/debian/meu_script.log
screen -S velocidade -X screen "rosrun beginner_tutorials beginner_tutorials_node_listener; exec bash"
echo "$(date) - Fim executando velocidade" >> /home/debian/meu_script.log


tmux new-session -d -s minha_sessao "roscore"
if [ $? -ne 0 ]; then
  echo "O comando ls falhou." >> /home/debian/meu_script.log
else
  echo "O comando ls foi executado com sucesso." >> /home/debian/meu_script.log
fi

#!/bin/bash
tmux new-session -d -s roscore "roscore"
tmux new-session -d -s imu "rosrun imu imu_node"
tmux new-session -d -s equilibrio "rosrun equilibrio equilibrio_node"
tmux new-session -d -s velocidade "rosrun beginner_tutorials beginner_tutorials_node_listener"


tmux split-window -t minha_sessao:0 -h "sleep 10; rosrun imu imu_node"


tmux new-session -d -s minha_sessao1 "sleep 5; rosrun equilibrio equilibrio_node"
tmux split-window -t minha_sessao1:0 -h "sleep 5; rosrun beginner_tutorials beginner_tutorials_node_listener"



#!/bin/bash
sleep 1
tmux new -s roscore -d 'roscore'
sleep 20
tmux new -s imu -d 'rosrun imu imu_node'
sleep 15
tmux new -s equilibrio -d 'rosrun equilibrio equilibrio_node'
sleep 15
tmux new -s velocidade -d 'rosrun beginner_tutorials beginner_tutorials_node_listener'

#!/bin/bash
sleep 1
roscore &
sleep 20
rosrun imu imu_node &
sleep 15
rosrun equilibrio equilibrio_node &
sleep 15
rosrun beginner_tutorials beginner_tutorials_node_listener &
