#!/usr/bin/env python
import argparse
import sys
import math

import rospy

import baxter_interface

def main():
    """Intento para mover el brazo del Baxter
    """
    print('Iniciando nodo...')
    rospy.init_node('nodo_mueve_brazo')
    brazoIzq = baxter_interface.Limb('left')
    brazoDer = baxter_interface.Limb('right')
    estado = baxter_interface.RobotEnable()
    estado.enable()
    while not rospy.is_shutdown():
        print 'Moviendo brazos a posicion 0'
        brazoIzq.move_to_neutral()
        #brazoDer.move_to_neutral()
        posicion = {'e0':0, 'e1':0, 's0':0, 's1':0, 'w0':0, 'w1':0, 'w2':0}
        posicionIzq = {}
        posicionDer = {}
        for i, j in posicion.iteritems():
            posicionIzq['left_'+i] = j
            posicionDer['right_'+i] = j
        brazoIzq.move_to_joint_positions(posicionIzq, 3)
        #brazoDer.move_to_joint_positions(posicionDer)


        for i, j in posicionIzq.iteritems():
            print 'moviendo brazo %s a pi y volviendo' % i
            posicionIzq[i] = math.pi
            brazoIzq.move_to_joint_positions(posicionIzq, 3)
            posicionIzq[i] = 0
            brazoIzq.move_to_joint_positions(posicionIzq, 3)

        break

if __name__ == '__main__':
    main()
