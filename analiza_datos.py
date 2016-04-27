#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand


def main():
    bag_rec = rosbag.Bag('recibido_v1.0_lapse100.bag')
    bag_sent = rosbag.Bag('enviado_v1.0_lapse100.bag')

    posicion = list()
    velocidad = list()
    torque = list()
    t = list()
    comando = list()

    try:
        for topic, msg, tiempo in bag_rec.read_messages(topics='/robot/joint_states'):
            posicion.append(msg.position)
            velocidad.append(msg.velocity)
            torque.append(msg.effort)
            t.append(tiempo.to_sec())
        for mode, msg, names in bag_sent.read_messages(topics='/robot/limb/left/joint_command'):
            comando.append(msg.command)

        t = [tiempo - t[0] for tiempo in t]
        print len(t), len(comando)

        plt.plot(t[1:len(comando)], velocidad[1:len(comando)], t[1:len(comando)], comando[1:len(comando)])
        plt.show()


    finally:
        bag_rec.close()
        bag_sent.close()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
