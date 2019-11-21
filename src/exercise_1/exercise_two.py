"""

title:        exercise one (Koordinatensysteme und Transformationen)

description:  Zum Rechnen mit Rotations-und Transformationsmatrizen sind
              folgende Funktionen in Python zu definieren.
              Benutzen Sie dazu den Datentyp array aus demPaket numpy.

author:       Tom Georgi, Florian Djokaj, David Wolpers
date:         17.10.2019

"""
import math
import numpy as np
import src.exercise_1.exercise_one as ex
from mpl_toolkits import mplot3d
import matplotlib.pyplot as pyp


x_R = 2
y_R = 1
theta = 30
l = 0.6
h = 0.2
r = 0.1
a = 0.1
b = 0.1
alpha = 40
l_one = 0.5
beta_one = 30
l_two = 0.5
beta_two = -10


def exercise_2_a():
    # t_O_R
    t_O_R = ex.create_transformationmatrix_z(theta, (x_R, y_R, r))

    # t_R_DB
    t_R_DB = ex.create_transformationmatrix_z(0, (l / 2 - a / 2, 0, h))

    # t_DB_D, Rotation um 2 Achsen
    tv_DB_D = (0, 0, b / 2)
    tv_DB_D = ex.trans(tv_DB_D)
    r_DB_D_z = ex.rot_z(alpha)
    r_DB_D_z = ex.rot_to_translate(r_DB_D_z)
    r_DB_D_x = ex.rot_x(90)
    r_DB_D_x = ex.rot_to_translate(r_DB_D_x)
    t_DB_D = np.linalg.multi_dot((tv_DB_D, r_DB_D_z, r_DB_D_x))

    # t_D_A1, Verschiebung um 2 Vektoren
    tv_D_A1_1 = (0, 0, a / 2)
    tv_D_A1_1 = ex.trans(tv_D_A1_1)
    r_D_A1_z = ex.rot_z(beta_one)
    r_D_A1_z = ex.rot_to_translate(r_D_A1_z)
    tv_D_A1_2 = (l_one, 0, 0)
    tv_D_A1_2 = ex.trans(tv_D_A1_2)
    t_D_A1 = np.linalg.multi_dot((tv_D_A1_1, r_D_A1_z, tv_D_A1_2))

    # t_A1_A2, erst matirx dann vektor
    tv_A1_A2 = (l_two, 0, 0)
    tv_A1_A2 = ex.trans(tv_A1_A2)
    r_A1_A2_z = ex.rot_z(beta_two)
    r_A1_A2_z = ex.rot_to_translate(r_A1_A2_z)
    t_A1_A2 = np.dot(r_A1_A2_z, tv_A1_A2)

    # t_O_A2
    t_O_D = np.linalg.multi_dot((t_O_R, t_R_DB, t_DB_D))
    t_O_A2 = np.linalg.multi_dot((t_O_D, t_D_A1, t_A1_A2))
    p_A2 = [0, 0, 0, 1]
    p_O = np.dot(t_O_A2, p_A2)
    print('p_O: \n', p_O)


def exercise_2_b():
    pass


def backwards_kinematic(point: tuple):
    x, y, z = point

    alpha_tmp = np.arctan2(y, (x - l / 2))
    alpha_tmp = np.rad2deg(alpha_tmp)

    t_R_DB = ex.create_transformationmatrix_z(0, (l / 2, 0, h))

    tv_DB_D = (0, 0, 0)
    tv_DB_D = ex.trans(tv_DB_D)
    r_DB_D_z = ex.rot_z(alpha_tmp)
    r_DB_D_z = ex.rot_to_translate(r_DB_D_z)
    r_DB_D_x = ex.rot_x(90)
    r_DB_D_x = ex.rot_to_translate(r_DB_D_x)
    t_DB_D = np.linalg.multi_dot((tv_DB_D, r_DB_D_z, r_DB_D_x))

    t_O_D = np.linalg.multi_dot((t_R_DB, t_DB_D))

    t_D_O = np.linalg.inv(t_O_D)
    p_D = np.dot(t_D_O, [x, y, z, 1])

    x, y, z = (p_D[0], p_D[1], p_D[2])

    a_tmp = math.sqrt((math.pow(x, 2) + math.pow(y, 2)))
    c_tmp = (math.pow(a_tmp, 2) - math.pow(l_one, 2) - math.pow(l_two, 2)) / (2 * l_one)
    b_tmp = -1 * math.sqrt(np.absolute((math.pow(l_two, 2) - math.pow(c_tmp, 2))))

    beta_two_tmp = np.rad2deg(np.arctan2(b_tmp, c_tmp))
    beta_one_tmp = np.rad2deg(np.arctan2(y, x) - np.arctan2(b_tmp, (l_one + c_tmp)))

    # print("Alpha: ", str(alpha_tmp))
    # print("Beta One: ", str(beta_one_tmp))
    # print("Beta Two: ", str(beta_two_tmp))

    forward_kinematic(alpha_tmp, beta_one_tmp, beta_two_tmp)
    return alpha_tmp, beta_one_tmp, beta_two_tmp


def forward_kinematic(alpha1, beta_one1, beta_two1):
    # t_O_R
    t_O_R = ex.create_transformationmatrix_z(theta, (x_R, y_R, r))

    # t_R_DB
    t_R_DB = ex.create_transformationmatrix_z(0, (l / 2, 0, h))

    # t_DB_D, Rotation um 2 Achsen
    tv_DB_D = (0, 0, 0)
    tv_DB_D = ex.trans(tv_DB_D)
    r_DB_D_z = ex.rot_z(alpha1)
    r_DB_D_z = ex.rot_to_translate(r_DB_D_z)
    r_DB_D_x = ex.rot_x(90)
    r_DB_D_x = ex.rot_to_translate(r_DB_D_x)
    t_DB_D = np.linalg.multi_dot((tv_DB_D, r_DB_D_z, r_DB_D_x))

    # t_D_A1, Verschiebung um 2 Vektoren
    tv_D_A1_1 = (0, 0, 0)
    tv_D_A1_1 = ex.trans(tv_D_A1_1)
    r_D_A1_z = ex.rot_z(beta_one1)
    r_D_A1_z = ex.rot_to_translate(r_D_A1_z)
    tv_D_A1_2 = (l_one, 0, 0)
    tv_D_A1_2 = ex.trans(tv_D_A1_2)
    t_D_A1 = np.linalg.multi_dot((tv_D_A1_1, r_D_A1_z, tv_D_A1_2))

    # t_A1_A2, erst matirx dann vektor
    tv_A1_A2 = (l_two, 0, 0)
    tv_A1_A2 = ex.trans(tv_A1_A2)
    r_A1_A2_z = ex.rot_z(beta_two1)
    r_A1_A2_z = ex.rot_to_translate(r_A1_A2_z)
    t_A1_A2 = np.dot(r_A1_A2_z, tv_A1_A2)

    # t_O_A2
    t_O_D = np.linalg.multi_dot((t_O_R, t_R_DB, t_DB_D))
    t_O_A2 = np.linalg.multi_dot((t_O_D, t_D_A1, t_A1_A2))
    p_A2 = [0, 0, 0, 1]
    p_O = np.dot(t_O_A2, p_A2)
    #print('p_O: \n', p_O)

    return p_O[0], p_O[1], p_O[2]


def exercise2_c():
    alpha1 = []
    beta_one1 = []
    beta_two1 = []
    y = []
    z = []
    x = []
    x2 = []
    y2 = []
    z2 = []

    for i in range(0, 360):
        yValue = 0.2 + 0.1 * math.cos(np.deg2rad(i))
        zValue = 0.2 + 0.1 * math.sin(np.deg2rad(i))
        alpha1, beta_one1, beta_two1, x, y, z, x2, y2, z2 = getAngle((1.2, yValue, zValue), alpha1, beta_one1, beta_two1,
                                                                     x, y, z, x2, y2, z2)

    pyp.plot(alpha1, 'b')
    pyp.plot(beta_one1, 'r')
    pyp.plot(beta_two1, 'y')
    pyp.legend(['alpha', 'beta1', 'beta2'])
    pyp.show()

    fig = pyp.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)

    fig2 = pyp.figure()
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.scatter(x2, y2, z2, c='r')
    pyp.show()


def getAngle(point, alpha1, beta_one1, beta_two1, x, y, z, x2, y2, z2):
    alphaReturn, beta_oneReturn, beta_twoReturn = backwards_kinematic(point)
    alpha1.append(alphaReturn)
    beta_one1.append(beta_oneReturn)
    beta_two1.append(beta_twoReturn)
    x.append(point[0])
    y.append(point[1])
    z.append(point[2])

    xValue, yValue, zValue = forward_kinematic(alphaReturn, beta_oneReturn, beta_twoReturn)
    x2.append(xValue)
    y2.append(yValue)
    z2.append(zValue)

    return alpha1, beta_one1, beta_two1, x, y, z, x2, y2, z2


if __name__ == '__main__':
    """ main """
    exercise_2_a()
    exercise2_c()

    backwards_kinematic((0.99163013, 0.58034659, 0.62101007))
    forward_kinematic(40, 30, -10)
