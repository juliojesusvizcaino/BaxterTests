#!/usr/bin/env python

import rospy
import math

import baxter_tools
import baxter_interface
import rosbag
from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def registro_datos(datos, bag_rec):
    bag_rec.write('/robot/joint_states', datos)

def inicia_baxter(pub, msg):
    msg.mode = JointCommand.POSITION_MODE
    msg.names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    msg.command = [0, 0, 0, 0, math.pi/2, 0, 0]

    pub.publish(msg)

def main():
    """Moviendo el brazo de Baxter mediante ordenes de ROS
    """
    frecuencia = 100
    print "Iniciando nodo"
    rospy.init_node("nodo_mueve_brazo_ROS")
    print "Creando editor (publisher) a %sHz" % frecuencia
    pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, tcp_nodelay=True, queue_size=1)
    rate = rospy.Rate(frecuencia)
    print "Creando registro"
    bag_sent = rosbag.Bag('enviado.bag', 'w')
    bag_rec = rosbag.Bag('recibido.bag', 'w')

    i = 0.0
    msg = JointCommand()
    msg.mode = JointCommand.POSITION_MODE
    msg.names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    msg.command = [0, 0, 0, 0, math.pi/2, 0, 0]

    print "Iniciando posicion Baxter"
    brazo = baxter_interface.Limb('left')
    pos_izq = dict()
    for contador in range(len(msg.names)):
        pos_izq[msg.names[contador]] = msg.command[contador]
    brazo.move_to_joint_positions(pos_izq)


    print "Creando suscriptor"
    rospy.Subscriber('/robot/joint_states', JointState, registro_datos, callback_args=bag_rec)
    try:
        print 'Empezando bucle'
        while not rospy.is_shutdown():
            msg.command[5] = math.sin(2*i)
            msg.command[1] = math.sin(i)
            pub.publish(msg)
            bag_sent.write('/robot/limb/left/joint_command', msg)
            i += 0.01
            rate.sleep()

    finally:
        bag_sent.close()
        bag_rec.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
