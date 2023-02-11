import rospy
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray


def callback(data):
    # Adicione o dado a um array para plotar
    global count
    count = count + 1

    x.append(len(x))
    velocidade.append(data.data[0])
    somador.append(data.data[1])
    referencia.append(data.data[2])
    error.append(data.data[3])

    # Limpe e redessine o gráfico
    if count >= 20:
        plt.clf()
        plt.plot(x, velocidade, color='y')
        plt.plot(x, somador, color='b')
        plt.plot(x, referencia, color='g')
        plt.plot(x, error, color='r')
        plt.pause(0.01)
        count = 0


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Float32MultiArray, callback)
    plt.show()
    rospy.spin()


if __name__ == '__main__':
    
    count = 0

    x = []
    velocidade = []
    somador = []
    referencia = []
    error = []

    count = 0

    listener()
