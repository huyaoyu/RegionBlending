
from __future__ import print_function

import cv2
import math
import numpy as np

INPUT_IMAGE = "MultiImageBlendingDirectly_Backup.jpg"

if __name__ == "__main__":
    H = np.eye(3, dtype = np.float)

    theta = 0.11383735513207814
    H[0, 0] = math.cos(theta); H[0, 1] = -math.sin(theta)
    H[1, 0] = math.sin(theta); H[1, 1] =  math.cos(theta)
    
    # Read the image.
    img = cv2.imread(INPUT_IMAGE, cv2.IMREAD_COLOR)

    # Warp the image.
    imgR = cv2.warpPerspective( img, H, (img.shape[1], img.shape[0]) )

    # Write image.
    cv2.imwrite("imgR.jpg", imgR)

    print("Done with writing imgR.")

    # Decompose matrix H.
    H[0, 2] = 1.0
    ret, R, T, N = cv2.decomposeHomographyMat(H, np.eye(3, dtype = np.float))

    print(R)
    print(T)
    print(N)
