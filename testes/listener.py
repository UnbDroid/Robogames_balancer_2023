#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import time


def callback_referencia0(data):
    # Adicione o dado a um array para plotar
    delay = time.time() - inicio    

    if(delay < 15):
        global count_velocidade
        count_velocidade = count_velocidade + 1

        x_velocidade.append(len(x_velocidade))
        referenceVelocidade.append(data.data[0])
        referencePosicao.append(data.data[1])
        referencia0.append(data.data[2])

        # Limpe e redessine o gráfico
        if count_velocidade >= 20:
            plt.subplot(211)
            plt.plot(x_velocidade, referenceVelocidade, color='r')
            plt.plot(x_velocidade, referencePosicao, color='b')
            plt.plot(x_velocidade, referencia0, color='m')
            plt.subplots_adjust(hspace=0.5)
            plt.pause(0.01)
            count_velocidade = 0

def callback_referencia1(data):
    # Adicione o dado a um array para plotar
    delay = time.time() - inicio    

    if(delay < 15):
        global count_posicao
        count_posicao = count_posicao + 1

        x_posicao.append(len(x_posicao))
        referencia1.append(data.data[3])

        # Limpe e redessine o gráfico
        if count_posicao >= 20:
            plt.subplot(212)
            plt.plot(x_posicao, referenceVelocidade, color='r')
            plt.plot(x_posicao, referencePosicao, color='b')
            plt.plot(x_posicao, referencia1, color='m')
            plt.subplots_adjust(hspace=0.5)
            plt.pause(0.01)
            count_posicao = 0

def callback_referencia2(data):
    # Adicione o dado a um array para plotar
    delay = time.time() - inicio    

    if(delay < 15):
        global count_posicao
        count_posicao = count_posicao + 1

        x_posicao1.append(len(x_posicao1))
        referencia2.append(data.data[4])

        # Limpe e redessine o gráfico
        if count_posicao >= 20:
            plt.subplot(213)
            plt.plot(x_posicao1, referenceVelocidade, color='r')
            plt.plot(x_posicao1, referencePosicao, color='b')
            plt.plot(x_posicao1, referencia2, color='m')
            plt.subplots_adjust(hspace=0.5)
            plt.pause(0.01)
            count_posicao = 0

def callback_referencia3(data):
    # Adicione o dado a um array para plotar
    delay = time.time() - inicio    

    if(delay < 15):
        global count_posicao
        count_posicao = count_posicao + 1

        x_posicao2.append(len(x_posicao2))
        referencia3.append(data.data[5])

        # Limpe e redessine o gráfico
        if count_posicao >= 20:
            plt.subplot(214)
            plt.plot(x_posicao2, referenceVelocidade, color='r')
            plt.plot(x_posicao2, referencePosicao, color='b')
            plt.plot(x_posicao2, referencia3, color='m')
            plt.subplots_adjust(hspace=0.5)
            plt.pause(0.01)
            count_posicao = 0


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Float32MultiArray, callback_referencia0)
    rospy.Subscriber("chatter", Float32MultiArray, callback_referencia1)
    rospy.Subscriber("chatter", Float32MultiArray, callback_referencia2)
    rospy.Subscriber("chatter", Float32MultiArray, callback_referencia3)


    plt.show()
    rospy.spin()


if __name__ == '__main__':

    inicio = time.time()
    
    count_velocidade = 0
    count_posicao = 0

    x_velocidade = []
    x_posicao = []
    x_posicao1 = []
    x_posicao2 = []
    theta = []
    velocidade = []
    posicao = []
    omega = []
    referencia = []
    referenceVelocidade = []
    referencePosicao = []

    referencia0 = []
    referencia1 = []
    referencia2 = []
    referencia3 = []

    listener()
