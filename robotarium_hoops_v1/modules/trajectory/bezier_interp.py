import numpy as np
from scipy.misc import factorial as ft
from scipy.misc import comb as nCk

#=====================================================================================================

def calc_bez_coeffs(T, p0, p1):
    # Bezier curve interpolation
    # T-conversion factor, dp/dt = dp/ds*1/T
    # p0, p1 are start/end points: x, xd, xdd, xddd, xdddd
    d = 4 # number of derivatives of p0, p1
    n = 9 # degree of curve
    Cout = np.zeros((n+1,3))
    
    # note: interpolation is done in [0,1], then converted to [0,T]
    for i in range(3):
        # cal x,y,z dimensions seperately
        CL = np.zeros((d+1,(n+1)/2))
        CR = np.zeros((d+1,(n+1)/2))

        x0 = np.array([p0[0,i],p0[1,i]*T,p0[2,i]*T**2,p0[3,i]*T**3,p0[4,i]*T**4])
        x1 = np.array([p1[0,i],p1[1,i]*T,p1[2,i]*T**2,p1[3,i]*T**3,p1[4,i]*T**4])

        CL[:,0] = np.multiply(ft(n-np.arange(d+1))/ft(n),x0)
        for j in range(d):
            CL[0:d-j,j+1] = CL[0:d-j,j] + CL[1:d-j+1,j]

        CR[:,-1] = np.multiply(ft(n-np.arange(d+1))/ft(n),x1)
        for j in range(d):
            CR[0:d-j,-2-j] = CR[0:d-j,-2-j+1] - CR[1:d-j+1,-2-j+1]

        Cout[:,i] = np.transpose(np.hstack((CL[0,:],CR[0,:])))
    return Cout

#=====================================================================================================

def state_from_bez(T, t, Cin):
    # Extract nominal state from Bezier control points
    d = 4 # derivative degree of Cin
    n = 9 # degree of curve
    tn = min(t/T,1) # normalized time, [0,1]
    Ctemp = np.zeros((d+1,n+1))
    phat  = np.zeros((d+1,3))

    for i in range(3):
        # cal 3 dimensional nominal states
        Ctemp[0,:] = np.transpose(Cin[:,i])
        for j in range(d):
            if j == 0:
                Ctemp[  1,   0:-1] = Ctemp[j,   1:] - Ctemp[j,   0:-1]
            else:
                Ctemp[j+1, 0:-1-j] = Ctemp[j, 1:-j] - Ctemp[j, 0:-1-j]
        # cal 0-4th order derivatives
        for k in range(d+1):
            temp = 0
            for kk in range(n-k+1):
                Btemp = nCk(n-k,kk)*(tn**kk)*((1-tn)**(n-k-kk))
                temp  = temp + ft(n)/ft(n-k)*Ctemp[k,kk]*Btemp
            phat[k,i] = temp/T**k
    return phat



