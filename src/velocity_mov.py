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

class MueveBrazoVel(object):
    def registro_rec(self, data):
        self.datos = data
        self.bag_rec.write('/robot/joint_states', data)

    def registro_sent(self, data):
        self.bag_sent.write(self.topic, data)

    def __init__(self, limb, frecuencia=100, velocity=1, intervalo=100):
        """
        Mueve el brazo de Baxter especificado mediante ordenes de ROS publicando
        la velocidad
        """
        self.limb = limb
        self.velocidad = velocity
        self.datos = JointState()
        self.intervalo = intervalo

        rospy.loginfo("Creando editor (publisher) a %sHz" % frecuencia)
        self.topic = '/robot/limb/'+self.limb+'/joint_command'
        self.pub = rospy.Publisher(self.topic, JointCommand, tcp_nodelay=True, queue_size=1)

        rospy.loginfo("Creando registros bag")
        nombre_rec = 'recibido_v' + str(self.velocidad) + '_lapse' + str(self.intervalo)
        self.bag_rec = rosbag.Bag(nombre_rec+'.bag', 'w')
        nombre_sent = 'enviado_v' + str(self.velocidad) + '_lapse' + str(self.intervalo)
        self.bag_sent = rosbag.Bag(nombre_sent+'.bag', 'w')

        rospy.loginfo("Creando suscriptores (subscriber)")
        rospy.Subscriber('/robot/joint_states', JointState, self.registro_rec)
        rospy.Subscriber(self.topic, JointCommand, self.registro_sent)

        self.msg = JointCommand()
        self.msg.mode = JointCommand.VELOCITY_MODE
        self.msg.names = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        for i, nombre in enumerate(self.msg.names):
            self.msg.names[i] = limb + '_' + nombre

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

    def mueve_brazo(self):
        i = 0.0
        pos = 1
        vel_range = [-self.velocidad, self.velocidad]
        w1_range = [-math.pi/2, 120.0/180*math.pi]
        self.msg.command = [0, 0, 0, 0, 0, 0, 0]

        rospy.loginfo('Empezando bucle')
        while not rospy.is_shutdown() and 0.9*w1_range[0] < self.datos.position[8] and self.datos.position[8] < 0.9*w1_range[1]:
            self.msg.command[5] = vel_range[pos]*1
            self.pub.publish(self.msg)
            i += 1
            if i < self.intervalo:
                pos = 1
            elif i < 2*self.intervalo:
                pos = 0
            else:
                i = 0
                pos = 1
            self.rate.sleep()

    def apagado(self):
        self.bag_rec.close()
        self.bag_sent.close()






def main():
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('-v', '--velocity', type=float, help='Velocidad del brazo (0,2)', default=1.0)
    parser.add_argument('-i', '--intervalo', type=int, help='Intervalo de tiempo que va hacia cada lado', default=100)
    args = parser.parse_args()

    velocidad = args.velocity
    intervalo = args.intervalo
    frecuencia = 100

    rospy.loginfo("Iniciando nodo")
    rospy.init_node("nodo_mueve_brazo_ROS")
    brazo_vel = MueveBrazoVel('left', frecuencia, velocidad, intervalo)

    brazo_vel.inicia_pos()

    brazo_vel.mueve_brazo()

    rospy.on_shutdown(brazo_vel.apagado)





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
