#!/usr/bin/env python

import rospy
import math
import argparse
import numpy as np

import baxter_tools
import baxter_interface
import rosbag
from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class MueveBrazoTorque(object):
    def registro_rec(self, data):
        self.datos = data
        self.bag_rec.write('/robot/joint_states', data)

    def registro_sent(self, data):
        self.bag_sent.write(self.topic, data)

    def __init__(self, limb='left', frecuencia=100, torque=10, intervalo=100):
        """
        Mueve el brazo de Baxter especificado mediante ordenes de ROS publicando
        el torque
        """
        self.limb = limb
        self.torque = torque
        self.datos = JointState()
        self.intervalo = intervalo
        self.alpha = 1

        rospy.loginfo("Creando editor (publisher) a %sHz" % frecuencia)
        self.topic = '/robot/limb/'+self.limb+'/joint_command'
        self.pub = rospy.Publisher(self.topic, JointCommand, tcp_nodelay=True, queue_size=1)

        rospy.loginfo("Creando registros bag")
        nombre_rec = 'recibido_v' + str(self.torque) + '_lapse' + str(self.intervalo)
        self.bag_rec = rosbag.Bag(nombre_rec+'.bag', 'w')
        nombre_sent = 'enviado_v' + str(self.torque) + '_lapse' + str(self.intervalo)
        self.bag_sent = rosbag.Bag(nombre_sent+'.bag', 'w')

        rospy.loginfo("Creando suscriptores (subscriber)")
        rospy.Subscriber('/robot/joint_states', JointState, self.registro_rec)
        rospy.Subscriber(self.topic, JointCommand, self.registro_sent)

        self.msg = JointCommand()
        self.msg.mode = JointCommand.TORQUE_MODE
        self.msg.names = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        for i, nombre in enumerate(self.msg.names):
            self.msg.names[i] = limb + '_' + nombre
        self.msg.command = list()

        self.rs = baxter_interface.RobotEnable()
        self.rs.enable()

        self.rate = rospy.Rate(frecuencia)



    def set_init(self):
        """ Establece posicion inicial con velocidaded (no funciona)"""
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
        brazo = baxter_interface.Limb(self.limb)
        command = [0, 0, 0, 0, math.pi/2, 0, 0]
        pos_izq = dict()
        for name, value in zip(self.msg.names, command):
            pos_izq[name] = value
        brazo.move_to_joint_positions(pos_izq)

    def definetorque(self, sentido, limites):
        torque_range = [-self.torque, self.torque]
        datoactual = self.datos.position[8]

        if sentido == 1 and datoactual < limites[1]:
            self.msg.command[5] = torque_range[sentido]
        elif sentido == 1:
            self.msg.command[5] = self.alpha*(limites[1] - datoactual)

        if sentido == 0 and datoactual > limites[0]:
            self.msg.command[5] = torque_range[sentido]
        elif sentido == 1:
            self.msg.command[5] = self.alpha*(limites[0] - datoactual)

    def mueve_brazo(self):
        i = 0.0
        sentido = 1
        w1_range = [-math.pi/2, 120.0/180*math.pi]
        pos0 = np.array([0, 0, 0, 0, math.pi/2, 0, 0])
        torque_actual = np.array(self.datos.effort[3:10])
        print self.msg.command

        rospy.loginfo('Empezando bucle')
        try:
            while not rospy.is_shutdown() and 0.9*w1_range[0] < self.datos.position[8] and self.datos.position[8] < 0.9*w1_range[1]:
                posicion_actual = np.array(self.datos.position[3:10])
                torque_actual = torque_actual + (pos0 - posicion_actual)
                self.msg.command = list(torque_actual)
                print self.datos.position[3:10]
                print list(pos0 - self.datos.position[3:10])
                print self.datos.effort[3:10]
                print ''
                #self.msg.command = torque_pos_inic
                #self.definetorque(sentido, w1_range)

                self.pub.publish(self.msg)
                i += 1
                if i < self.intervalo:
                    sentido = 1
                elif i < 2*self.intervalo:
                    sentido = 0
                else:
                    i = 0
                    sentido = 1
                self.rate.sleep()
        finally:
            self.bag_rec.close()
            self.bag_sent.close()





def main():
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('-t', '--torque', type=float, help='Torque del brazo (0,15)', default=10.0)
    parser.add_argument('-i', '--intervalo', type=int, help='Intervalo de tiempo que va hacia cada lado', default=100)
    args = parser.parse_args()

    torque = args.torque
    intervalo = args.intervalo
    frecuencia = 100

    rospy.loginfo("Iniciando nodo")
    rospy.init_node("nodo_mueve_brazo_ROS")
    brazo_vel = MueveBrazoTorque('left', frecuencia, torque, intervalo)

    brazo_vel.inicia_pos()

    brazo_vel.mueve_brazo()





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
