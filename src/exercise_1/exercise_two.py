"""

title:        exercise one (Koordinatensysteme und Transformationen)

description:  Zum Rechnen mit Rotations-und Transformationsmatrizen sind
              folgende Funktionen in Python zu definieren.
              Benutzen Sie dazu den Datentyp array aus demPaket numpy.

author:       Tom Georgi, Florian Djokaj, David Wolpers
date:         17.10.2019

"""
import numpy as np
import exercise_one as ex


def exercise_2_a():
    # t_O_R
    t_O_R = ex.create_transformationmatrix_z(0, (0, 0, 0.1))

    # t_R_DB
    t_R_DB = ex.create_transformationmatrix_z(0, (0.6 / 2 - 0.1 / 2, 0, 0.2))

    # t_DB_D, Rotation um 2 Achsen
    tv_DB_D = (0, 0, 0.1 / 2)
    tv_DB_D = ex.trans(tv_DB_D)
    r_DB_D_z = ex.rot_z(40)
    r_DB_D_z = ex.rot_to_translate(r_DB_D_z)
    r_DB_D_x = ex.rot_x(90)
    r_DB_D_x = ex.rot_to_translate(r_DB_D_x)
    t_DB_D = np.dot(tv_DB_D, r_DB_D_z, r_DB_D_x)

    # t_D_A1, Verschiebung um 2 Vektoren
    tv_D_A1_1 = (0, 0, 0.1 / 2)
    tv_D_A1_1 = ex.trans(tv_D_A1_1)
    r_D_A1_z = ex.rot_z(30)
    r_D_A1_z = ex.rot_to_translate(r_D_A1_z)
    tv_D_A1_2 = (0.5, 0, 0)
    tv_D_A1_2 = ex.trans(tv_D_A1_2)
    t_D_A1 = np.dot(tv_D_A1_1, r_D_A1_z, tv_D_A1_2)

    # t_A1_A2, erst matirx dann vektor
    tv_A1_A2 = (0.5, 0, 0)
    tv_A1_A2 = ex.trans(tv_A1_A2)
    r_A1_A2_z = ex.rot_z(-10)
    r_A1_A2_z = ex.rot_to_translate(r_A1_A2_z)
    t_A1_A2 = np.dot(r_A1_A2_z, tv_A1_A2)

    # t_O_A2
    t_O_D = np.dot(t_O_R, t_R_DB, t_DB_D)
    t_O_A2 = np.dot(t_O_D, t_D_A1, t_A1_A2)
    p_A2 = [0, 0, 0, 1]
    p_O = np.dot(t_O_A2, p_A2)
    print('p_O: \n', p_O)


#def exercise_2_b():


if __name__ == '__main__':
    """ main """
    exercise_2_a()
    # exercise_2_2()
