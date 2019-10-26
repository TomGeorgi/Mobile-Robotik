"""

title:        exercise one (Koordinatensysteme und Transformationen)

description:  Zum Rechnen mit Rotations-und Transformationsmatrizen sind
              folgende Funktionen in Python zu definieren.
              Benutzen Sie dazu den Datentyp array aus demPaket numpy.

author:       Tom Georgi, Florian Djokaj, David Wolpers
date:         17.10.2019

"""
import numpy as np


def trans(t):
    """
    Function returns a homogeneous Translation matrix
    with the translation t.

    Args:
        t (tuple): Translation with the size of 2 (x, y) or 3 (x, y, z)

    Returns:
        np.array of a homogeneous Translation matrix.
    """
    if len(t) == 2:
        arr = np.identity(2)
        arr = np.append(arr, np.vstack(t), axis=1)
        arr = np.append(arr, [[0, 0, 1]], axis=0)
    else:
        arr = np.identity(3)
        arr = np.append(arr, np.vstack(t), axis=1)
        arr = np.append(arr, [[0, 0, 0, 1]], axis=0)

    return arr


def rot(theta):
    """
    returns a 2D rotation matrix with rotation angle theta.

    Args:
        theta (int): rotation angle

    Returns:
        np.array of a 2D rotation matrix
    """
    theta = np.radians(theta)
    return np.array([[np.cos(theta), - np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def rot_x(theta):
    """
    returns an elementary 3D rotation matrix with rotation
    angle theta around rotation axis x.

    Args:
        theta (int): rotation angle

    Returns:
        np.array of a 3D rotation matrix around the rotation axis x.
    """
    theta = np.radians(theta)
    return np.array([[1, 0, 0], [0, np.round(np.cos(theta), 2), - np.round(np.sin(theta), 2)],
                     [0, np.round(np.sin(theta), 2), np.round(np.cos(theta), 2)]])


def rot_y(theta):
    """
    returns an elementary 3D rotation matrix with rotation
    angle theta around rotation axis y.

    Args:
        theta (int): rotation angle

    Returns:
        np.array of a 3D rotation matrix around the rotation axis y.
    """
    theta = np.radians(theta)
    return np.array([[np.round(np.cos(theta), 2), 0, np.round(np.sin(theta), 2)], [0, 1, 0],
                     [np.round(- np.sin(theta), 2), 0, np.round(np.cos(theta), 2)]])


def rot_z(theta):
    """
    returns an elementary 3D rotation matrix with rotation
    angle theta around rotation axis z.

    Args:
        theta (int): rotation angle

    Returns:
        np.array of a 3D rotation matrix around the rotation axis z.
    """
    theta = np.radians(theta)
    return np.array([[np.round(np.cos(theta), 2), - np.round(np.sin(theta), 2), 0],
                     [np.round(np.sin(theta), 2), np.round(np.cos(theta), 2), 0], [0, 0, 1]])


def rot_to_translate(r):
    """
    converts the rotation matrix r into a
    homogeneous transformation matrix

    Args:
        r (np.array): rotation matrix

    Raises:
        AttributeError if the shape is not (2, 2) or (3, 3)

    Returns:
        np.array of a homogeneous transformation matrix
    """
    arr = r
    if r.shape == (2, 2):
        arr = np.append(arr, np.vstack((0, 0)), axis=1)
        arr = np.append(arr, [[0, 0, 1]], axis=0)
    elif r.shape == (3, 3):
        arr = np.append(arr, np.vstack((0, 0, 0)), axis=1)
        arr = np.append(arr, [[0, 0, 0, 1]], axis=0)
    else:
        raise AttributeError("Wrong shape!")

    return arr


def create_transformationmatrix_z(theta, t_vektor):
    t_vektor = trans(t_vektor)
    r_matrix = rot_z(theta)
    r_matrix = rot_to_translate(r_matrix)
    return np.dot(t_vektor, r_matrix)


def create_transformationmatrix_2d(theta, t_vektor):
    t_vektor = trans(t_vektor)
    r_matrix = rot(theta)
    r_matrix = rot_to_translate(r_matrix)
    return np.dot(t_vektor, r_matrix)


def exercise_2_1():
    # a)
    tv_a_b = (-2, 0, 0)
    t_a_b = create_transformationmatrix_z(180, tv_a_b)
    print("T_a_b:", t_a_b)

    tv_b_c = (-4, -1, 0)
    t_b_c = create_transformationmatrix_z(-90, tv_b_c)
    print("T_b_c:", t_b_c)

    tv_a_c = (2, 1, 0)
    t_a_c = create_transformationmatrix_z(90, tv_a_c)
    print("T_a_c:", t_a_c)

    # b)
    t_a_c_2 = np.dot(t_a_b, t_b_c)
    print("b) ", np.array_equal(t_a_c, t_a_c_2))

    # c)
    t_c_a_2 = np.linalg.inv(t_a_c)
    tv_c_a = (-1, 2, 0)
    t_c_a = create_transformationmatrix_z(-90, tv_c_a)
    print("c) ", np.array_equal(t_c_a_2, t_c_a))

    # d)
    p_b = [-3, 1, 0, 1]
    p_a = np.dot(t_a_b, p_b)
    print("p_a:", p_a)


def exercise_2_2():
    # a)
    tv_0_a = (1, 1)
    t_0_a = create_transformationmatrix_2d(0, tv_0_a)
    print("T_0_a:", t_0_a)

    tv_0_b = (3, 2)
    t_0_b = create_transformationmatrix_2d(30, tv_0_b)
    print("T_0_b:", t_0_b)

    # b)
    p_b = [1, 1, 1]
    p_0 = np.dot(t_0_b, p_b)
    print("P_0:", p_0)

    # c)
    t_a_0 = np.linalg.inv(t_0_a)
    t_a_b = np.dot(t_a_0, t_0_b)
    print("T_a_b:", t_a_b)

    # d)
    p_a = np.dot(t_a_b, p_b)
    print("P_a:", p_a)

    # e)
    p_a_a = np.dot(t_a_b, p_a)
    print("p_a_a:", p_a_a)


if __name__ == '__main__':
    """ main """
    exercise_2_1()
    exercise_2_2()
