from math import sqrt, atan2
from math import pi as PI
from numpy.linalg import norm

#=====================================================================================================

def invert_diff_flat_output(x, thrust_hover=49201):
    m = 35.89/1000
    g = 9.8

    beta1 = -x[2,0]
    beta2 = -x[2,2] + 9.8
    beta3 =  x[2,1]

    roll    = atan2(beta3, sqrt(beta1**2 + beta2**2)) * (180/PI)
    pitch   = atan2(beta1, beta2) * (180/PI)
    yawrate = 0

    a_temp = norm([0, 0, g] - x[2,:])
    thrust = int(a_temp/g * thrust_hover)

    return roll, pitch, yawrate, thrust



