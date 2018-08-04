import numpy as np
from numpy.linalg import norm
from math import exp

#=====================================================================================================

def time_dilation(phat, x, s, dt):
    # Time dialation/Virtual time
    ks   = 100
    sd   = exp(-ks*norm(phat[0,:] - x[0,:])**2) #paramed time dynamics
    pd   = phat[1,:]*sd
    sdd  = -ks*exp(-ks*norm(phat[0,:] - x[0,:])**2)*2*np.dot(phat[0,:] - x[0,:],pd-x[1,:]) 
    pdd  = phat[2,:]*sd**2 + phat[1,:]*sdd
    sddd = ks*ks*exp(-ks*norm(phat[0,:]-x[0,:])**2)*4*(np.dot(phat[0,:]-x[0,:],pd-x[1,:]))**2-ks*2*exp(-ks*norm(phat[0,:]-x[0,:])**2)*(np.dot(pd-x[1,:],pd-x[1,:])+np.dot(phat[0,:]-x[0,:],pdd-x[2,:])) 
    pddd = phat[3,:]*sd**3 + 3*phat[2,:]*sd*sdd + phat[1,:]*sddd
    pdddd= phat[4,:]*sd**4 # approximate
    phatnew = np.vstack([phat[0,:], pd, pdd, pddd, pdddd])
    s = s + sd*dt

    return phatnew, s



