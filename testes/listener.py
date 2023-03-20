import rospy
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import time



def callback(data):
    # Adicione o dado a um array para plotar

    delay = time.time() - inicio    

    if(delay < 15):
        global count
        count = count + 1

        x.append(len(x))
        angulo.append(data.data[0])
        referencia.append(data.data[1])

        # Limpe e redessine o gráfico
        if count >= 20:
            plt.clf()
            plt.plot(x, angulo, color='r')
            plt.plot(x, referencia, color='b')
            plt.pause(0.01)
            count = 0


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Float32MultiArray, callback)
    plt.show()
    rospy.spin()


if __name__ == '__main__':

    inicio = time.time()
    
    count = 0

    x = []
    angulo = []
    referencia = []

    listener()
