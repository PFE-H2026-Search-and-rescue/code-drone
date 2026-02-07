# Final transform for UI only: world = R * local + T
# R is a 2x2 matrix, T is a 2-vector. Initially identity transform.
R = [[1,0],[0,1]]
T = (0,0)
CALIBRATED = False  # flag used to switch between local and calibrated world coords

def get_R():
    global R
    return R

def get_T():
    global T
    return T


def set_R(new_r):
    global R
    R = new_r

def set_T(new_t):
    global T
    T = new_t

def get_calib():
    global CALIBRATED
    return CALIBRATED

def set_calib(new_calib):
    global CALIBRATED
    CALIBRATED = new_calib
