#!/usr/bin/env python

import rospy
import math

import baxter_tools
import baxter_interface
import rosbag
from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def main():
    """Moviendo el brazo de Baxter mediante ordenes de ROS
    """
    frecuencia = 100
    print "Iniciando nodo"
    rospy.init_node("nodo_mueve_brazo_ROS")
    print "Creando editor (publisher) a %sHz" % frecuencia
    pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, tcp_nodelay=True, queue_size=1)
    rate = rospy.Rate(frecuencia)

    i = 0.0
    intervalo = 1000
    pos = 0
    w1_range = [-math.pi/2, 120.0/180*math.pi]

    msg = JointCommand()
    msg.mode = JointCommand.TORQUE_MODE
    msg.names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    msg.command = [0, 0, 0, 0, math.pi/2, 0, 0]

    print "Iniciando posicion Baxter"
    brazo = baxter_interface.Limb('left')
    pos_izq = dict()
    for contador in range(len(msg.names)):
        pos_izq[msg.names[contador]] = msg.command[contador]
    brazo.move_to_joint_positions(pos_izq)

    print 'Empezando bucle'
    while not rospy.is_shutdown():
        msg.command[5] = w1_range[pos]*1
        pub.publish(msg)
        i += 1
        if i < intervalo:
            pos = 0
        elif i < 2*intervalo:
            pos = 1
        else:
            i = 0
            pos = 0
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
