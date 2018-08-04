import numpy as np
from math import sin, cos, sqrt
from numpy.linalg import norm, inv

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d

#=====================================================================================================

class CrazyfliePlot():
    def __init__(self, ax, initpos, roll=0.0, pitch=0.0, yaw=0.0, trans=1.0):
        self.pos  = initpos
        self.traj = np.zeros((0,3))
        self.traj = np.vstack((self.traj, self.pos))
        self.ptraj, = plt.plot(self.traj[:,0] , self.traj[:,1], self.traj[:,2], 'k--')

        self.r0 = 0.08
        self.r1 = 0.06

        L = 0.35
        R = self.RotMat()
        d = np.cross((R[0,2], R[1,2], R[2,2]),(0, 0, 1)) #Obtain the rotation vector    
        self.M = rotation_matrix(d) #Get the rotation matrix

        self.hx, = plt.plot([initpos[0] , initpos[0]+L*R[0,0]], [initpos[1] , initpos[1]+L*R[1,0]], [initpos[2] , initpos[2]+L*R[2,0]], 'r', alpha=min(1,trans*2))
        self.hy, = plt.plot([initpos[0] , initpos[0]+L*R[0,1]], [initpos[1] , initpos[1]+L*R[1,1]], [initpos[2] , initpos[2]+L*R[2,1]], 'g', alpha=min(1,trans*2))
        self.hz, = plt.plot([initpos[0] , initpos[0]+L*R[0,2]], [initpos[1] , initpos[1]+L*R[1,2]], [initpos[2] , initpos[2]+L*R[2,2]], 'b', alpha=min(1,trans*2))

        self.disk1 = Circle((initpos[0]+self.r0/sqrt(2), initpos[1]+self.r0/sqrt(2)), self.r1, alpha=trans)
        self.disk2 = Circle((initpos[0]+self.r0/sqrt(2), initpos[1]-self.r0/sqrt(2)), self.r1, alpha=trans)
        self.disk3 = Circle((initpos[0]-self.r0/sqrt(2), initpos[1]+self.r0/sqrt(2)), self.r1, alpha=trans)
        self.disk4 = Circle((initpos[0]-self.r0/sqrt(2), initpos[1]-self.r0/sqrt(2)), self.r1, alpha=trans)

        ax.add_patch(self.disk1)
        ax.add_patch(self.disk2)
        ax.add_patch(self.disk3)
        ax.add_patch(self.disk4)

        pathpatch_2d_to_3d(self.disk1, z=initpos[2], normal=((R[0,2],R[1,2],R[2,2])))
        pathpatch_2d_to_3d(self.disk2, z=initpos[2], normal=((R[0,2],R[1,2],R[2,2])))
        pathpatch_2d_to_3d(self.disk3, z=initpos[2], normal=((R[0,2],R[1,2],R[2,2])))
        pathpatch_2d_to_3d(self.disk4, z=initpos[2], normal=((R[0,2],R[1,2],R[2,2])))


    def RotMat(self, r=0.0,p=0.0,y=0.0):
        R = np.array([[cos(p)*cos(y), sin(r)*sin(p)*cos(y)-cos(r)*sin(y), cos(r)*sin(p)*cos(y)+sin(r)*sin(y)],
                      [cos(p)*sin(y), sin(r)*sin(p)*sin(y)+cos(r)*cos(y), cos(r)*sin(p)*sin(y)-sin(r)*cos(y)],
                      [-sin(p), sin(r)*cos(p), cos(r)*cos(p)]])
        return R


    def update(self, pos, r, p, y=0):
        R = self.RotMat(r, p, y)
        L = 0.35
        self.hx.set_xdata([pos[0] , pos[0]+L*R[0,0]])
        self.hx.set_ydata([pos[1] , pos[1]+L*R[1,0]])
        self.hx.set_3d_properties([pos[2] , pos[2]+L*R[2,0]])
        self.hy.set_xdata([pos[0] , pos[0]+L*R[0,1]])
        self.hy.set_ydata([pos[1] , pos[1]+L*R[1,1]])
        self.hy.set_3d_properties([pos[2] , pos[2]+L*R[2,1]])
        self.hz.set_xdata([pos[0] , pos[0]+L*R[0,2]])
        self.hz.set_ydata([pos[1] , pos[1]+L*R[1,2]])
        self.hz.set_3d_properties([pos[2] , pos[2]+L*R[2,2]])
        d = np.cross((R[0,2], R[1,2], R[2,2]),(0, 0, 1)) #Obtain the rotation vector    
        newM = rotation_matrix(d) #Get the rotation matrix
        self.disk1._segment3d -= np.kron(np.ones((self.disk1._segment3d[:,0].size,1)), self.pos)
        self.disk2._segment3d -= np.kron(np.ones((self.disk2._segment3d[:,0].size,1)), self.pos)
        self.disk3._segment3d -= np.kron(np.ones((self.disk3._segment3d[:,0].size,1)), self.pos)
        self.disk4._segment3d -= np.kron(np.ones((self.disk4._segment3d[:,0].size,1)), self.pos)
        self.disk1._segment3d = np.dot(newM, np.dot(inv(self.M), self.disk1._segment3d.T)).T
        self.disk2._segment3d = np.dot(newM, np.dot(inv(self.M), self.disk2._segment3d.T)).T
        self.disk3._segment3d = np.dot(newM, np.dot(inv(self.M), self.disk3._segment3d.T)).T
        self.disk4._segment3d = np.dot(newM, np.dot(inv(self.M), self.disk4._segment3d.T)).T
        self.disk1._segment3d += np.kron(np.ones((self.disk1._segment3d[:,0].size,1)), pos)
        self.disk2._segment3d += np.kron(np.ones((self.disk2._segment3d[:,0].size,1)), pos)
        self.disk3._segment3d += np.kron(np.ones((self.disk3._segment3d[:,0].size,1)), pos)
        self.disk4._segment3d += np.kron(np.ones((self.disk4._segment3d[:,0].size,1)), pos)
        self.pos = pos
        self.M = newM

        # update trajectory
        self.traj = np.vstack((self.traj, pos))
        self.ptraj.set_xdata(self.traj[:,0])
        self.ptraj.set_ydata(self.traj[:,1])
        self.ptraj.set_3d_properties(self.traj[:,2])


def rotation_matrix(d):
    """
    Calculates a rotation matrix, M, given a vector, d:
    Direction of d corresponds to the rotation axis
    Length of d corresponds to the sine of the angle of rotation

    Variant of: http://mail.scipy.org/pipermail/numpy-discussion/2009-March/040806.html
    """
    sin_angle = norm(d)

    if sin_angle == 0:
        return np.identity(3)

    d /= sin_angle

    eye  = np.eye(3)
    ddt  = np.outer(d, d)
    skew = np.array([[    0,  d[2], -d[1]],
                     [-d[2],     0,  d[0]],
                     [ d[1], -d[0],    0]], dtype=np.float64)

    M = ddt + np.sqrt(1 - sin_angle**2) * (eye - ddt) + sin_angle * skew
    return M


def pathpatch_2d_to_3d(pathpatch, z = 0, normal = 'z'):
    """
    Transforms a 2D Patch to a 3D patch using the given normal vector.

    The patch is projected into they XY plane, rotated about the origin
    and finally translated by z.
    """
    if type(normal) is str: #Translate strings to normal vectors
        index = "xyz".index(normal)
        normal = np.roll((1.0,0,0), index)

    normal /= norm(normal) #Make sure the vector is normalised

    path = pathpatch.get_path() #Get the path and the associated transform
    trans = pathpatch.get_patch_transform()

    path = trans.transform_path(path) #Apply the transform

    pathpatch.__class__ = art3d.PathPatch3D #Change the class
    pathpatch._code3d = path.codes #Copy the codes
    pathpatch._facecolor3d = pathpatch.get_facecolor #Get the face color    

    verts = path.vertices #Get the vertices in 2D

    d = np.cross(normal, (0, 0, 1)) #Obtain the rotation vector    
    M = rotation_matrix(d) #Get the rotation matrix

    pathpatch._segment3d = np.array([np.dot(M, (x, y, 0)) + (0, 0, z) for x, y in verts])



