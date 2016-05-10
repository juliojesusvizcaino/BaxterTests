#!/usr/bin/env python

import rospy
import math
import argparse
import numpy as np
import time
from collections import deque
import matplotlib.pyplot as plt

import baxter_tools
import baxter_interface
import rosbag
from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState

class MueveTorqueRange(object):
    def registro_rec(self, data):
        self.datos = data
        for i, name in enumerate(self.msg.names):
            idx = [j for j, name2 in enumerate(data.name) if name==name2]
            self.position[i] = data.position[idx[0]]
            self.velocity[i] = data.velocity[idx[0]]
            self.effort[i] = data.effort[idx[0]]

        if self.guarda:
            self.bag_rec.write('/robot/joint_states', data)

    def registro_sent(self, data):
        if self.guarda:
            self.bag_sent.write(self.topic, data)

    def __init__(self, limb, frecuencia=100, torque=1):
        """
        Mueve el brazo de Baxter especificado mediante ordenes de ROS publicando
        la velocidad
        """
        self.limb = limb
        self.torque = torque
        self.datos = JointState()
        self.torquemax = [50, 50, 50, 50, 15, 15, 15]
        self.guarda = False
        self.position = [None]*7
        self.velocity = [None]*7
        self.effort = [None]*7

        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        rospy.loginfo("Creando editor (publisher) a %sHz" % frecuencia)
        self.topic = '/robot/limb/'+self.limb+'/joint_command'
        self.pub = rospy.Publisher(self.topic, JointCommand, tcp_nodelay=True, queue_size=1)
        self.topicGravity = '/robot/limb/'+self.limb+'/suppress_gravity_compensation'
        self.pubGravity = rospy.Publisher(self.topicGravity, Empty, tcp_nodelay=True, queue_size=1)

        rospy.loginfo("Creando registros bag")
        nombre_rec = 'recibido_torque' + str(self.torque)
        self.bag_rec = rosbag.Bag(nombre_rec+'.bag', 'w')
        nombre_sent = 'enviado_torque' + str(self.torque)
        self.bag_sent = rosbag.Bag(nombre_sent+'.bag', 'w')

        rospy.loginfo("Creando suscriptores (subscriber)")
        rospy.Subscriber('/robot/joint_states', JointState, self.registro_rec)
        rospy.Subscriber(self.topic, JointCommand, self.registro_sent)

        self.msg = JointCommand()
        self.msg.mode = JointCommand.TORQUE_MODE
        self.msg.names = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        for i, nombre in enumerate(self.msg.names):
            self.msg.names[i] = limb + '_' + nombre

        self.msgGravity = Empty()

        self.rs = baxter_interface.RobotEnable()
        self.rs.enable()

        self.rate = rospy.Rate(frecuencia)

    def set_init(self):
        """ Establece posicion inicial con velocidades (no funciona)"""
        posfinal = [0, 0, 0, 0, math.pi/2, 0, 0]
        pos0 = np.array(posfinal)
        names = self.datos.name
        self.rate.sleep()

        def extract_pos(name):
            def posiciones():
                pos = [i for i, x in enumerate(self.datos.name) if x == name]
                pos = pos[0]
                return self.datos.position[pos]
            return posiciones

        pos = [extract_pos(i) for i in self.msg.names]

        def pos_update(posiciones):
            return [posiciones[i]() for i in range(len(posiciones))]

        posiciones = pos_update(pos)

        rospy.loginfo('Iniciando posicion Baxter')
        while np.sum((pos0 - posiciones)**2) > 0.3:
            posiciones = np.array(pos_update(pos))
            self.msg.command = list((pos0 - posiciones)*10)
            self.pub.publish(self.msg)
            self.rate.sleep()

        rospy.sleep(1.)

    def inicia_pos(self):
        rospy.loginfo('Iniciando posicion')
        brazo = baxter_interface.Limb(self.limb)
        command = [0, 0, 0, 0, math.pi/2, 0, 0]
        pos_izq = dict()
        for name, value in zip(self.msg.names, command):
            pos_izq[name] = value
        brazo.move_to_joint_positions(pos_izq)

    def compruebatorque(self, torque):
        for i, valor in enumerate(torque):
            if np.abs(valor) > self.torquemax[i]:
                torque[i] = np.sign(valor)*self.torquemax[i]
        return torque

    def filtrapasobaja(self, torques):
        torques_out = list()
        for valores in zip(*torques):
            torques_out.append(np.mean(valores))
        return torques_out

    def mueve_brazo(self):
        i = 0.0
        pos = 1
        w1_range = [-math.pi/2, 120.0/180*math.pi]
        #self.msg.command = [0, 0, 0, 0, 0, 0, 0]
        self.msg.command = self.effort
        time.sleep(2.)
        posicion_inicial = np.array([0, 0, 0, 0, math.pi/2, 0, 0])
        torque_com = deque([])
        [torque_com.append(self.msg.command) for _ in range(0,20)]

        rospy.loginfo('Empezando bucle')
        self.guarda = True
        while not rospy.is_shutdown():
            pos_actual = np.array(self.position)
            vel_actual = np.array(self.velocity)
            #torque_com.append(np.array(self.msg.command) + (posicion_inicial - pos_actual)*0.1)
            torque_com.append(-((posicion_inicial - pos_actual)^2 + vel_actual^2)
            torque_com.popleft()
            #torque_sent = self.filtrapasobaja(torque_com)
            torque_sent = list(torque_com[-1])
            #self.msg.command = self.compruebatorque(torque_sent)
            self.msg.command = torque_sent

            self.pub.publish(self.msg)
            #self.pubGravity.publish(self.msgGravity)
            self._pub_cuff_disable.publish()
            self.rate.sleep()

    def apagado(self):
        self.bag_rec.close()
        self.bag_sent.close()






def main():
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('-t', '--torque', type=float, help='Torque del brazo (0,15)', default=1.0)
    args = parser.parse_args()

    torque = args.torque
    frecuencia = 100

    rospy.loginfo("Iniciando nodo")
    rospy.init_node("nodo_mueve_brazo_ROS")
    brazo_vel = MueveTorqueRange('left', frecuencia, torque)

    brazo_vel.inicia_pos()

    brazo_vel.mueve_brazo()

    rospy.on_shutdown(brazo_vel.apagado)





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
