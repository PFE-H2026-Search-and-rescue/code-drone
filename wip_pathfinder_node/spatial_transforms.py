import numpy as np

# def get_3d_transform(P, Q):
#     """
#     Finds similarity transform (s, R, t) that maps P to Q: Q = sRP + t
#     Input: P, Q (N x 3 matrices)
#     Output: 4x4 Transformation Matrix
#     """
#     # 1. Centroids
#     centroid_P = np.mean(P, axis=0)
#     centroid_Q = np.mean(Q, axis=0)

#     # 2. Center the data
#     Pp = P - centroid_P
#     Qq = Q - centroid_Q

#     # 3. Scale factor
#     scale = np.sum(np.linalg.norm(Qq, axis=1)) / np.sum(np.linalg.norm(Pp, axis=1))

#     # 4. Rotation using SVD
#     H = Pp.T @ Qq
#     U, S, Vt = np.linalg.svd(H)
#     R = Vt.T @ U.T

#     # Special reflection case
#     if np.linalg.det(R) < 0:
#         Vt[2, :] *= -1
#         R = Vt.T @ U.T

#     # 5. Translation
#     t = centroid_Q - scale * (R @ centroid_P)

#     # 6. Build 4x4 matrix
#     M = np.eye(4)
#     M[0:3, 0:3] = scale * R
#     M[0:3, 3] = t
#     return M

def convert_from_object_to_vector2(vector_size, object):
    if(vector_size == 1): #robot
        return np.array([object["x"], object["y"]])
    elif(vector_size == 2): #drone
        return np.array([object["x"], object["z"]])

def add_to_matrix(vector, matrix, is_drone):
    chiffre = 1
    if is_drone :
        chiffre = 2

    vector_array = convert_from_object_to_vector2(chiffre, vector)
    matrix = np.vstack((matrix,vector_array))#np.c_[matrix, vector_array] 
    return matrix

def transform_point(M, x, y):
    """
    Applies a 2x3 affine matrix M to a single point (x, y).
    """
    # M[row, col]
    new_x = M[0, 0] * x + M[0, 1] * y + M[0, 2]
    new_y = M[1, 0] * x + M[1, 1] * y + M[1, 2]
    
    return (new_x, new_y)

def estimate_transform(src, dst):
    # src and dst are (N, 2) arrays of points
    n = src.shape[0]
    
    # Set up the linear system Ax = b
    A = np.zeros((2 * n, 4))
    b = np.zeros((2 * n))
    
    for i in range(n):
        x, y = src[i]
        u, v = dst[i]
        A[2*i]   = [x, -y, 1, 0]
        A[2*i+1] = [y,  x, 0, 1]
        b[2*i]   = u
        b[2*i+1] = v
        
    # Solve for [a, b, tx, ty]
    params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    a, b_val, tx, ty = params
    
    # Construct 2x3 Affine Matrix
    M = np.array([[a, -b_val, tx],
                  [b_val,  a, ty]])
    
    # Extract physical properties
    # scale = np.sqrt(a**2 + b_val**2)
    # angle = np.degrees(np.arctan2(b_val, a))
    
    return M#, scale, angle, (tx, ty)