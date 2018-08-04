#!/usr/bin/env python

import sys
import time
import pickle
import rospy

import numpy as np
from math import pi as PI
from numpy.linalg import norm
from control import acker

import matplotlib.pyplot  as plt

from modules import ros_objects as ros
from modules import sim_plotting as sim
from modules import trajectory as traj

#=================================================================================================

# SETUP:
Nf = 1          # Number of 'flies (restricted to 1 for now...)
cf_offset = 0   # ROS id offset -- for when you're not starting with "crazyflie0"
Nh = 3          # Number of hoops
hoop_offset = 0 # ROS id offset -- for when you're not starting with "hoop0"
dt = 0.02       # Time-step (used in simulation and execution)
# Desired speed is currently set in traj.path_plan (default: 0.8)

# OPTIONS:
track_stat = False # Obtaining data from VRPN? Simulate with desired initial conditions if FALSE
show_sim   = True  # Show simulated trajectory
sim_prog   = True  # Display simulation progress (command-line messages)
execute    = False # Execute on the Robotarium (overridden if track_stat == False)

#=================================================================================================

if __name__ == "__main__":

    #=============================================================================================
    # INPUT HANDLING
    #=============================================================================================

    #=======================================================================#
    # Path planning specification:                                          #
    #    - Entry Format: "[hoop num][hoop side] [hoop num][hoop side] ..."  #
    #    - Hoop numbers: from 0 to (total hoops - 1)                        #
    #    - Hoop side: F = front, R = rear                                   #
    #    - Example: "1F 2R 2F" (Spaces req'd between entries for parsing)   #
    #=======================================================================#

    if len(sys.argv) == 1:
        # Default command-string:
        cmd_string = "1F 2R 2F"
        # Other "test" cases:
        #cmd_string = "0F 2F 2R"
        #cmd_string = "0F 1F 2F 1R"
        #cmd_string = "0F 0R 0F 0R 0F"
        #cmd_string = "2R 2R 0R 1F 2R 1R 1F"

    elif len(sys.argv) == 2:
        # Command-string provided via command-line:
        cmd_string = str(sys.argv[1])

    else:
        print 'Invalid number of arguments (pass as a single string. ex: "1F 2R 2F")'
        sys.exit()

    # Create list from input string:
    cmd_list = cmd_string.split(" ")

    # Check for errors in input:
    for i in range(len(cmd_list)):
        if (len(cmd_list[i]) != 2):
            print "Input String Error! Entry number", i+1, "-->", cmd_list[i], "(invalid length)"
            sys.exit()
        if (int(cmd_list[i][0]) > Nh-1):
            print "Input String Error! Entry number", i+1, "-->", cmd_list[i], "(invalid hoop num.)"
            sys.exit()
        if (cmd_list[i][1] != 'F') and (cmd_list[i][1] != 'R'):
            print "Input String Error! Entry number", i+1, "-->", cmd_list[i], "(invalid hoop side)"
            sys.exit()

    #=============================================================================================
    # INITIALIZATION
    #=============================================================================================

    p_init = dict()  # Stores initial position(s) of the crazyflie(s)
    p_hover = dict() # Stores hovering position(s)    "    "    "
    center = dict()  # Stores center position(s) of the hoop(s)

    cfs = dict()     # Stores instances of crazyflie "ROS objects"
    hoops = dict()   # Stores instances of hoop "ROS objects"

    if (track_stat == True):
        # Create ROS node:
        rospy.init_node('robotarium_hoops', anonymous = True)

        # Create and store a "ROS object" for each crazyflie:
        for i in range(Nf):
            cfs[i] = ros.CrazyflieObj(i + cf_offset)
            # Record initial position from tracking data:
            p_init[i] = [cfs[i].position[0], cfs[i].position[1], cfs[i].position[2]]

        # Create and store a "ROS object" for each hoop:
        for i in range(Nh):
            hoops[i] = ros.HoopObj(i + hoop_offset, track_stat)
            # Record initial position from tracking data:
            center[i] = hoops[i].center

    else:
        # Pick your own initial conditions for simulation:
        p_init[0] = np.array([ 1.45,  0.0,  0.0])

        center[0] = np.array([  0.8,  0.0, -0.6])
        center[1] = np.array([  0.0,  0.0, -0.6])
        center[2] = np.array([ -0.8,  0.0, -0.6])

        # Currently, the "ROS object" is used even when ROS will not be needed... This is done
        # to reuse the code for "helper point" generation (front, rear, avoid pts). This block
        # needs to be reworked.
        for i in range(Nh):
            hoops[i] = ros.HoopObj(i + hoop_offset, track_stat)

    for i in range(Nf):
        print "'flie", i, "p_init =", p_init[i]
        # Set hovering-positions based on initial positions (xinit, yinit, z=-0.8)
        p_hover[i] = [p_init[i][0], p_init[i][1], -0.8]

    for i in range(Nh):
        print "hoop ", i, "center =", center[i]

    #=============================================================================================
    # PATH PLANNING
    #=============================================================================================

    wpts = dict()  # Dictionary to store waypoint seq
    T_list = []    # List of times to spend in each trajectory segment

    wpts, T_list = traj.gen_wpt_seq(p_hover[0], hoops, cmd_list)

    #=============================================================================================
    # SIMULATION
    #=============================================================================================

    if show_sim == True:
        # Figure setup:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_aspect('equal')
        ax.set_axis_off()

        # Add crazyflie to plot:
	CfCoord = sim.CrazyfliePlot(ax, wpts[0][0,:])

        # Add hoops to plot:
        for i in range(Nh):
            sim.HoopPlot(ax, hoops[i].center, hoops[i].radius, hoops[i].front, hoops[i].rear, hoops[i].avoid)

        # Add arena to plot:
        sim.ArenaPlot(ax, 2.0, 1.5, 2.0) # (axes handle, arena dimensions)
        plt.pause(.001)

    x    = dict() # Actual state
    x[0] = wpts[0][0:-1,:]   

    xd    = dict() # Derivative of x
    xd[0] = np.zeros((4,3))

    uhat    = dict() # Control input
    uhat[0] = np.zeros((1,3)) 

    phat    = dict() # Nominal state from interpolation (x stacked on top of uhat)
    phat[0] = np.zeros((5,3)) 

    t_ind   = 0   # Time index
    t_real  = 0.0 # Current experiment time
    T_total = 0.0 # Length of experiment

    for i in range(len(T_list)):
        T_total += T_list[i]

    seg_ind = 0   # Segment index

    tseg_i  = 0   # Segment timestep index
    t_seg   = 0.0 # Time spent in current segment
    T_curr  = 0.0 # Time to complete current segment (also used as a scaling factor)

    s = 0.0       # Virtual time
    flag_done = 1 # Indicates completion of segment

    Cout = np.zeros((10,3)) # Bezier interpolation coefficients

    # Pole placement for CLF and CBF
    AA=np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0]])
    bb=np.array([[0], [0], [0], [1]]) 
    Kb=np.asarray(acker(AA,bb,[-12.2,-12.4,-12.6,-12.8]))

    # Dictionaries for holding simulation results:
    x_sim = dict()
    u_sim = dict()

    print "Starting Simulation! (", len(T_list), "segments )"
    while (t_real < T_total):
        if flag_done == 1 and seg_ind < (len(T_list)):
            T_curr = T_list[seg_ind]
            # Bezier interpolation between waypoints:
            Cout = traj.calc_bez_coeffs(T_curr, wpts[seg_ind], wpts[seg_ind+1])
            s = 0
            tseg_i = 0
            flag_done = 0
            seg_ind += 1
            print "Seg.", seg_ind, "complete"

        t_real = t_ind*dt
        t_seg  = tseg_i*dt

        # Extract current nominal state from Bezier coefficients:
        phat[0] = traj.state_from_bez(T_curr, s, Cout)
                    
        # Check if waypoint finished:
        if t_seg >= T_curr and s > T_curr:
            flag_done = 1

        # Time dilation to smooth out aggressive manuevers
        phat[0], s = traj.time_dilation(phat[0], x[0], s, dt)

        # Compute nominal control:
        # xd = AA*x + bb*u, where: u = ref(4) - Kb*(x-ref)
        uhat[0] = phat[0][4,:] - np.dot(Kb, (x[0] - phat[0][0:-1,:]))
        # Bound the norm of the resulting control signal:
        if norm(uhat[0]) > 1e4:
            uhat[0] = uhat[0]/norm(uhat[0])*1e4
        u = uhat.copy()

        # Compute feedforward attitude controls (estimated min-thrust used):
        roll, pitch, yawrate, thrust = traj.invert_diff_flat_output(x[0])

        # Update "frame" coordinates for plotting:
        if show_sim == True:
            CfCoord.update(x[0][0,:], roll*(PI/180), pitch*(PI/180), 0)

        # Update actual dynamics:
        xd[0] = np.dot(AA, x[0]) + np.dot(bb, u[0])
        x[0]  = x[0] + xd[0]*dt  

        # Record simulated state and input:
        x_sim[t_ind] = x.copy()
        u_sim[t_ind] = u.copy()             

        # Increment time indicies:
        t_ind  += 1
        tseg_i += 1

        # Only update plot after every fifth time step:
        if show_sim == True: 
            if t_ind%5 == 1:
                plt.pause(.001)

    print "Simulation Finished!"
    # Keep plot displayed after simulation finishes:
    plt.show()

    # Proceed or kill?
    if (execute == False) or (track_stat == False):
        sys.exit()

    #=============================================================================================
    # EXECUTION
    #=============================================================================================

    # Blocking array (used to make sure all CFs are "ready" before proceeding to next stage)
    fflag = np.zeros((Nf,1))

    #----------#
    # HOVERING #
    #----------#

    # Record starting time, use 3s to take off and hover:  
    t_start   = rospy.Time.now()
    t_takeoff = 3.0

    s = 0 # Parameterized time [0~1]
    while (s < 1) or (sum(fflag) < Nf):
        t_curr = rospy.Time.now()
        s = min((t_curr - t_start).to_sec()/t_takeoff, 1.0) # s = 0 --> 1
        for i in range(Nf):
            cfs[i].send_cmd_diff()
            if cfs[i].state != 3:
                cfs[i].goto(p_hover[i])
                t_start = rospy.Time.now()
            else:
                s = max(s, 0.5)
                fflag[i] = cfs[i].gotoT(p_hover[i], s)  

    time.sleep(2)

    # Recording hovering thrust (future work: average multiple thrust readings)
    thrust_hover = dict()
    for i in range(Nf):
        thrust_hover[i] = cfs[i].cmd_vel.linear.z 
        cfs[i].send_cmd_diff(0, 0, 0, thrust_hover[i])        

    # Reset blocking array:
    fflag = np.zeros((Nf,1))

    #----------------#
    # FIRST WAYPOINT #
    #----------------#

    # Record start time, use 3s to reach first waypoints:
    t_start  = rospy.Time.now()
    t_waypts = 3.0

    s = 0 #parameterized time [0~1]
    while (s < 1) or (sum(fflag) < Nf):
        #cfs[i].send_cmd_diff(0, 0, 0, thrust_hover[i])
        t_curr = rospy.Time.now()
        s = min((t_curr - t_start).to_sec()/t_waypts, 1.0)
        for i in range(Nf):
            cfs[i].send_cmd_diff(0, 0, 0, thrust_hover[i])
            fflag[i] = cfs[i].gotoT(p_hover[i], s)
    time.sleep(2)

    #----------------#
    # SIM "PLAYBACK" #
    #----------------#

    # Trajectory and control input at a specific time:
    x_t = dict()
    u_t = dict()

    # Tracking position and orientation at a specific time:
    xyz_t = dict()
    rpy_t = dict()

    # Live experiment data recording:
    exp_time_hist = dict()
    goal_dyn_hist = dict()
    xyztrack_hist = dict()
    rpytrack_hist = dict()
    
    t_start = rospy.Time.now()
    t_index = 0

    while not (rospy.is_shutdown() or t_index*dt >= T_total):
        try:
            # Record starting time of each loop iteration:
            iter_start = rospy.Time.now()

            # Load planned trajectory data:
            x_t = x_sim[t_index]
            u_t = u_sim[t_index]

            # Update localization and control:
            for i in range(Nf):
                cfs[i].updatepos()

                xyz_t[i] = cfs[i].position
                rpy_t[i] = cfs[i].orientation

                roll, pitch, yrate, thrust = traj.invert_diff_flat_output(x_t[i], thrust_hover[i])

                cfs[i].goto(x_t[i][0,:])                         # Send new goal (for PID)
                cfs[i].send_cmd_diff(roll, pitch, yrate, thrust) # Send feedforward term

            # Record experiment data:
            exp_time_hist[t_index] = (rospy.Time.now() - t_start).to_sec()
            goal_dyn_hist[t_index] = x_t.copy()
            xyztrack_hist[t_index] = xyz_t.copy()
            rpytrack_hist[t_index] = rpy_t.copy()

            t_index = t_index + 1

            # Pause to synchronize experiment with simulation's time steps:
            while (rospy.Time.now() - iter_start).to_sec() < dt:
                dummy_var = 1

        except rospy.ROSInterruptException:
            print '----Experiment interrupted!!!----'
            break

    print '----Experiment completed!!!----'

    # Save data:
    f = open('Hoop_Experiment.pckl', 'w')
    pickle.dump([exp_time_hist, goal_dyn_hist,  xyztrack_hist, rpytrack_hist], f)
    f.close()

