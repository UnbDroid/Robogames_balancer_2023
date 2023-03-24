#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import time



def callback_velocidade(data):
    # Adicione o dado a um array para plotar
    delay = time.time() - inicio    

    if(delay < 15):
        global count_velocidade
        count_velocidade = count_velocidade + 1

        x_velocidade.append(len(x_velocidade))
        velocidade.append(data.data[0])
        referenceVelocidade.append(data.data[2])

        # Limpe e redessine o gráfico
        if count_velocidade >= 20:
            plt.subplot(211)
            plt.plot(x_velocidade, referenceVelocidade, color='r')
            plt.plot(x_velocidade, velocidade, color='b')
            plt.subplots_adjust(hspace=0.5)
            plt.pause(0.01)
            count_velocidade = 0

def callback_posicao(data):
    # Adicione o dado a um array para plotar
    delay = time.time() - inicio    

    if(delay < 15):
        global count_posicao
        count_posicao = count_posicao + 1

        x_posicao.append(len(x_posicao))
        posicao.append(data.data[1])
        referencePosicao.append(data.data[3])

        # Limpe e redessine o gráfico
        if count_posicao >= 20:
            plt.subplot(212)
            plt.plot(x_posicao, referencePosicao, color='c')
            plt.plot(x_posicao, posicao, color='g')
            plt.subplots_adjust(hspace=0.5)
            plt.pause(0.01)
            count_posicao = 0


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Float32MultiArray, callback_posicao)
    rospy.Subscriber("chatter", Float32MultiArray, callback_velocidade)
    plt.show()
    rospy.spin()


if __name__ == '__main__':

    inicio = time.time()
    
    count_velocidade = 0
    count_posicao = 0

    x_velocidade = []
    x_posicao = []
    theta = []
    velocidade = []
    posicao = []
    omega = []
    referencia = []
    referenceVelocidade = []
    referencePosicao = []

    listener()