import rospy
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32


def callback(data):
    # Adicione o dado a um array para plotar
    count = count + 1
    x.append(len(x))
    y.append(data.data)
    # Limpe e redessine o gráfico
    if count >= 20 :
        plt.clf()
        plt.plot(x, y)
        plt.pause(0.01)
        count = 0


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Float32, callback)
    plt.show()
    rospy.spin()


if __name__ == '__main__':
    
    x = []
    y = []
    count = 0
    listener()
