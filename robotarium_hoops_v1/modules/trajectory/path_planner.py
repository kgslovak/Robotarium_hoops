import numpy as np
from math import sin, cos, atan2, sqrt
from math import pi as PI

#-----------------------------------------------------------------------------------------------------

spd_des = 0.8     # Desired average speed

#=====================================================================================================

def create_wpt(pos_wpt, spd_wpt = 0, dir_wpt = 0):

    # Waypoint format:
    #                  [[               x,               y,               z],
    #                   [              xd,              yd,              zd],
    #                   [             xdd,             ydd,             zdd],
    #                   [            xddd,            yddd,            zddd],
    #                   [           xdddd,           ydddd,           zdddd]]

    wpt_new = np.array([[      pos_wpt[0],      pos_wpt[1],      pos_wpt[2]],
                        [ dir_wpt*spd_wpt,             0.0,             0.0],
                        [             0.0,             0.0,             0.0],
                        [             0.0,             0.0,             0.0],
                        [             0.0,             0.0,             0.0]])
    return wpt_new

#=====================================================================================================

def append_wpt(wpts_curr, T_list_curr, pos_next, spd_next, dir_next):
    # Returned objects:
    wpts_new   = dict()
    T_list_new = []

    # Generate keys for accessing and appending waypoints dictionary:
    key_last = len(wpts_curr) - 1
    key_new  = key_last + 1

    # Copy and append waypoints dictionary:
    wpts_new = wpts_curr
    wpts_new[key_new] = create_wpt(pos_next, spd_next, dir_next)

    # Extract the direction of the last waypoint and calc inter-wpt distances:
    dir_last = np.sign(wpts_curr[key_last][1,0])

    dist_x   = abs(pos_next[0] - wpts_curr[key_last][0,0])
    dist_y   = abs(pos_next[1] - wpts_curr[key_last][0,1])
    dist_z   = abs(pos_next[2] - wpts_curr[key_last][0,2])
    dist_yz  = sqrt(dist_y**2 + dist_z**2)
    
    # Calculate approximate segment-length:
    if (dist_yz == 0):
        # segment shape: just a straight-away
        length_seg = dist_x
    elif (dir_next == dir_last):
        if (dist_x <= dist_yz):
            # segment shape: 2 quarter arcs joined by a straight-away (unless dist_x == dist_yz)
            length_seg = (dist_yz - dist_x) + (dist_x * PI/2)
        else:
            # segment shape: two equal length tangent arcs ("s-curve")
            rt_tri_leg = 0.25 * sqrt(dist_x**2 + dist_yz**2)
            theta = atan2(dist_yz, dist_x)
            r = rt_tri_leg / sin(theta)
            length_seg = 2*(r*(2*theta))
    else:
        # segment shape: half-circle preceeded by straight-away (unless dist_x == 0)
        length_seg = dist_x + (dist_y * PI/2)
    
    # Use "global" desired speed for seg-time calc if input speed is zero (such as the final wpt):
    if (spd_next == 0):
        spd_seg = spd_des
    else:
        spd_seg = spd_next

    # Calculate segment-time:
    if (length_seg == 0):
        T_seg = 1/(2*spd_seg)
    else:
        T_seg = length_seg/spd_seg

    # Copy and append segment-time list:
    T_list_new = T_list_curr
    T_list_new.append(T_seg)

    return wpts_new, T_list_new

#=====================================================================================================

def add_avoid_wpts(wpts_curr, T_list_curr, pos_curr, pos_next, hoops):
    # Returned objects:
    wpts_new   = dict()
    T_list_new = []

    # List for keeping track of hoops to avoid:
    avoid_list = []

    # Calculate direction from current point to next point:
    direction  = np.sign(pos_next[0] - pos_curr[0])

    # Generate list of hoops to avoid:
    for i in range(len(hoops)):
        if direction == -1:
            if (pos_curr[0] > hoops[i].center[0]) and (hoops[i].center[0] > pos_next[0]):
                avoid_list.append(i)
        elif direction == +1:
            if (pos_curr[0] < hoops[i].center[0]) and (hoops[i].center[0] < pos_next[0]):
                avoid_list.append(i)
        else:
            #print "pos_next same as pos_curr"
            break

    if direction == +1:
        avoid_list.reverse() #(Robotarium xdir is "flipped")

    # Copy then append waypoints dictionary and segment-time list:
    wpts_new = wpts_curr
    T_list_new = T_list_curr

    for j in range(len(avoid_list)):
        wpts_new, T_list_new = append_wpt(wpts_new, T_list_new, hoops[avoid_list[j]].avoid, spd_des, direction)

    return wpts_new, T_list_new

#=====================================================================================================

def add_thru_wpts(wpts_curr, T_list_curr, p_enter, p_exit):
    # Returned objects:
    wpts_new   = dict()
    T_list_new = []

    # Calculate direction through hoop:
    direction  = np.sign(p_exit[0] - p_enter[0])

    # Copy then append waypoints dictionary and segment-time list:
    wpts_new = wpts_curr
    T_list_new = T_list_curr

    wpts_new, T_list_new = append_wpt(wpts_new, T_list_new, p_enter, spd_des, direction)
    wpts_new, T_list_new = append_wpt(wpts_new, T_list_new, p_exit , spd_des, direction)

    return wpts_new, T_list_new

#=====================================================================================================

def gen_wpt_seq(pos_hover, hoops, cmd_list):
    # Returned objects:
    wpts = dict()  # Dictionary to store waypoint sequence (each wpt is a separate entry)
    T_list = []    # List of time to spend in each trajectory segment

    # First waypoint based on initial position of crazyflie (at hovering height):
    wpts[0] = create_wpt([pos_hover[0], pos_hover[1], pos_hover[2]])

    pos_curr = np.array([wpts[0][0,0], wpts[0][0,1], wpts[0][0,2]])

    for i in range(len(cmd_list)):
        # Extract id of next hoop to be passed through:
        h_next = int(cmd_list[i][0])

        # Extract starting point for pass-through based on specified hoop side:
        if cmd_list[i][1] == 'F':
            pos_next = hoops[h_next].front
        elif cmd_list[i][1] == 'R':
            pos_next = hoops[h_next].rear

        wpts, T_list = add_avoid_wpts(wpts, T_list, pos_curr, pos_next, hoops)

        # Set appropriate "current" point for next iteration
        if cmd_list[i][1] == 'F':
            wpts, T_list = add_thru_wpts(wpts, T_list, hoops[h_next].front, hoops[h_next].rear )
            pos_curr = hoops[h_next].rear

        elif cmd_list[i][1] == 'R':
            wpts, T_list = add_thru_wpts(wpts, T_list, hoops[h_next].rear,  hoops[h_next].front)
            pos_curr = hoops[h_next].front

    # Return to the initial starting position:
    pos_last = np.array([wpts[0][0,0], wpts[0][0,1], wpts[0][0,2]])

    wpts, T_list = add_avoid_wpts(wpts, T_list, pos_curr, pos_last, hoops)
    wpts, T_list = append_wpt(wpts, T_list, pos_last, 0, 0)

    return wpts, T_list



