from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d

#=====================================================================================================

class HoopPlot():
    def __init__(self, ax, center_pt, radius, front_pt, rear_pt, avoid_pt):
        p = Circle((center_pt[1], center_pt[2]), radius, color='g', fill=False)
        ax.add_patch(p)
        art3d.pathpatch_2d_to_3d(p, z=center_pt[0], zdir="x")

        ax.scatter(center_pt[0], center_pt[1], center_pt[2], color='g', marker='o')
        ax.scatter( front_pt[0],  front_pt[1],  front_pt[2], color='b', marker='o')
        ax.scatter(  rear_pt[0],   rear_pt[1],   rear_pt[2], color='y', marker='o')
        ax.scatter( avoid_pt[0],  avoid_pt[1],  avoid_pt[2], color='r', marker='o')
        #ax.scatter(avoid_pts[:,0], avoid_pts[:,1], avoid_pts[:,2], color='r', marker='o')



