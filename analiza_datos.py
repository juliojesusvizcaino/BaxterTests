#!/usr/bin/env python

import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os

from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand

class RepresentaBag(object):
    def __init__(self, nombre):
        self.nombre = nombre
        self.bag_rec = rosbag.Bag('recibido_'+self.nombre+'.bag')
        self.bag_sent = rosbag.Bag('enviado_'+self.nombre+'.bag')
        self.posicion = list()
        self.velocidad = list()
        self.torque = list()
        self.t = list()
        self.comando = list()

    def leedatos(self):
        for topic, msg, tiempo in self.bag_rec.read_messages(topics='/robot/joint_states'):
            self.posicion.append(msg.position)
            self.velocidad.append(msg.velocity)
            self.torque.append(msg.effort)
            self.t.append(tiempo.to_sec())
        for mode, msg, names in self.bag_sent.read_messages(topics='/robot/limb/left/joint_command'):
            self.comando.append(msg.command)

    def ajustatiempo(self):
        self.t = [tiempo - self.t[0] for tiempo in self.t]

    def igualalongitud(self):
        lonmin = min(len(self.t), len(self.comando))
        self.t = self.t[:lonmin]
        self.posicion = self.posicion[:lonmin]
        self.velocidad = self.velocidad[:lonmin]
        self.torque = self.torque[:lonmin]
        self.comando = self.comando[:lonmin]

    def muestra(self, parametro):
        self.fig = plt.figure(1)
        plt.plot(self.t, self.posicion, self.t, self.comando)
        plt.xlabel('Tiempo')
        plt.ylabel(parametro)
        plt.title(self.nombre)
        #plt.show()

    def guarda(self):
        self.fig.savefig('imagenes/'+self.nombre+'.png', format='png')


    def close(self):
        self.bag_rec.close()
        self.bag_sent.close()
        plt.close()


def main():
    nombres = [f for f in os.listdir('.') if f[:3] == 'env']
    for nombre in nombres:
        if nombre[8:9] == 'v':
            dato = 'velocidad'
        else:
            dato = 'posicion'
        img = RepresentaBag(nombre[8:-4])
        img.leedatos()
        img.ajustatiempo()
        img.igualalongitud()
        img.muestra(dato)
        img.guarda()
        img.close()
        rospy.on_shutdown(img.close)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
