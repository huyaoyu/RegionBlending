
from __future__ import print_function

import cv2
import math
import numpy as np

def show_matrix_list(ml, name = None):
    if ( name is not None ):
        print(name)

    for m in ml:
        print(m)
        print("")

if __name__ == "__main__":
    # Load H.
    H = np.loadtxt("H.dat", dtype = np.float)

    # H = np.eye(3, dtype = np.float)
    # theta = math.pi / 2
    # H[0, 0] = math.cos(theta); H[0, 1] = -math.sin(theta); H[0, 2] = 1.0
    # H[1, 0] = math.sin(theta); H[1, 1] =  math.cos(theta)

    # Dummy intrinsic matrix.
    K = np.eye(3, dtype = np.float)

    # Decompose the homography matrix.
    ret, R, T, N = cv2.decomposeHomographyMat( H, K )

    # Show the decomposed values.
    show_matrix_list(R, "R")
    show_matrix_list(T, "T")
    show_matrix_list(N, "N")

    # A test point.
    p0 = np.array([1, 0, 1], dtype = np.float).reshape((-1, 1))

    n = len(R)

    for i in range(n):
        H1 = R[i] + T[i].dot( N[i].transpose() )
        p1 = H1.dot(p0)
        print("i = %d" % (i))
        print(H1)
        print(p1 / p1[-1, 0] )

    # The true transformation.
    print("H * p0 = ")
    p0 = np.array([1, 0, 1], dtype = np.float).reshape((3, 1))
    p1 = H.dot(p0)
    print( p1 / p1[2, 0] )
