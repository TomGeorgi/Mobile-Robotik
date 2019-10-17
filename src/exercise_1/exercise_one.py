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
    return np.array([[1, 0, 0], [0, np.cos(theta), - np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])


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
    return np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [- np.sin(theta), 0, np.cos(theta)]])


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
    return np.array([[np.cos(theta), - np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])


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


# Exercise 2.1 a)
# t_a_b = np.array([[-1, 0, 0, -2], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
# t_b_c = np.array([[0, 1, 0, -4], [-1, 0, 0, -1], [0, 0, 1, 0], [0, 0, 0, 1]])
# t_a_c = np.array([[0, -1, 0, 2], [1, 0, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])

# Exercise 2.1 b)
# print(t_a_c)
# print(np.matmul(t_a_b, t_b_c))
# if np.array_equal(t_a_c, np.matmul(t_a_b, t_b_c)):
#     print("True")
# else:
#     print("False")

if __name__ == '__main__':
    """ main """
    # placeholder
    # print(rot_to_translate(np.identity(2)))
