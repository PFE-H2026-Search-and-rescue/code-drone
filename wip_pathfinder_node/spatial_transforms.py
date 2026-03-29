import numpy as np

def get_3d_transform(P, Q):
    """
    Finds similarity transform (s, R, t) that maps P to Q: Q = sRP + t
    Input: P, Q (N x 3 matrices)
    Output: 4x4 Transformation Matrix
    """
    # 1. Centroids
    centroid_P = np.mean(P, axis=0)
    centroid_Q = np.mean(Q, axis=0)

    # 2. Center the data
    Pp = P - centroid_P
    Qq = Q - centroid_Q

    # 3. Scale factor
    scale = np.sum(np.linalg.norm(Qq, axis=1)) / np.sum(np.linalg.norm(Pp, axis=1))

    # 4. Rotation using SVD
    H = Pp.T @ Qq
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Special reflection case
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # 5. Translation
    t = centroid_Q - scale * (R @ centroid_P)

    # 6. Build 4x4 matrix
    M = np.eye(4)
    M[0:3, 0:3] = scale * R
    M[0:3, 3] = t
    return M

def convert_from_object_to_vector3(vector_size, object):
    if(vector_size == 3):
        return np.array([object["x"], object["y"], object["z"]])
    elif(vector_size == 4):
        return np.array([object["x"], object["y"], object["z"], 1])

def add_to_matrix(vector, matrix):
    vector_array = convert_from_object_to_vector3(3, vector)
    matrix = np.vstack((matrix, vector_array)) 
    return matrix