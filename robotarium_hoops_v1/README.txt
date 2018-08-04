This package contains the python scripts required to run the hoop demo on the Robotarium

Based on work by Li Wang. Written by Kyle Slovak


===============================================================================================
Execution instructions:
===============================================================================================

  After launching the crazyflie_ros scripts which establish communication with the crazyflie
  and the tracking system, run "hoop_demo_v1.py" in a new terminal. Once the simulation has
  finished, the gamepad can be used to start execution.


===============================================================================================
File Directory Layout:
===============================================================================================

  robotarium_hoops_v1
    |-- README.txt
    |-- hoop_demo_v1.py
    '-- modules
          |-- __init__.py (blank)
          |-- ros_objects
          |     |-- __init__.py (import everything)
          |     |-- crazyflie_obj.py
          |     '-- hoop_obj.py
          |-- sim_plotting
          |     |-- __init__.py (import everything)
          |     |-- arena_plot.py
          |     |-- crazyflie_plot.py
          |     '-- hoop_plot.py
          '-- trajectory
                |-- __init__.py (import everything)
                |-- attitude_ctrl.py
                |-- bezier_interp.py
                |-- path_planner.py
                '-- time_dilation.py

===============================================================================================
Overviews:
===============================================================================================

  hoop_demo_v1.py
  ---------------
    Main script: Setup and run simulation (and execute on the Robotarium when tracking).

    Stages
      1. Imports
           - Contributed modules automatically load their respective submodules.

      2. Setup/Options 
           - Simulation/execution of trajectories is currently limited to a single quad.
             However, considerations were made ahead of time to prep for multi-agent cases.

      3. Input Handling
           - List of objectives read in from command-line parameters or from a 
             pre-programmed string.

           - Errors in the input are checked for prior to proceeding.              

      4. Initialization
           - Create objects which facilitate the interface with ROS (and consequently the
             access to motion tracking data).

           - If tracking is not available, run just the simulation based on hard-coded
             initial positions.

      5. Path Planning
           - First, convert the list of input objectives into a sequence of waypoints.
             Waypoints are 5x3 matricies containing:

               [[     x,     y,     z],
                [    xd,    yd,    zd],
                [   xdd,   ydd,   zdd],
                [  xddd,  yddd,  zddd],
                [ xdddd, ydddd, zdddd]]  *(num. of d's indicates order of the derivative)

             This is a truncated version of the 5x4 matrix containing x, y, z, yaw and
             their derivatives. For how we are modeling the quadcopter, x, y, z, and yaw are
             the selected differentially flat outputs. Since this program will zero-out yaw
             and its derivatives, we don't need to input this 4th column of information.

           - Waypoints are placed so that, after interpolation, the quadcopter flies through
             the hoops as specified and then returns to its starting position. The path 
             planning functions which contribute to this task include algorithms for avoiding
             "obstacle" hoops and calculating segment times between waypoints (to achieve a
             constant speed as much as possible).

      6. Simulation
           - If the user has enabled a plot of the simulation, the necessary set up is
             performed in this stage.

           - Using Bezier interpolation, 9th order splines are generated to connect
             consecutive waypoints in a piecewise manner -- as opposed to generating a single
             spline which is constrained at every waypoint. 

           - Code to implement time dilation and linear feedback control is currently included
             in this stage (which doesn't make much sense since these methods depend on the
             error between the desired state and actual state). Future work: move these blocks
             to the execution stage once real-time computation is implemented (currently just
             "playing back" the simulation).

           - Currently, the only check performed during simulation is on the magnitude of the
             norm of the control input. Future work: check current position with arena bounds

           - The resulting trajectory is then converted to attitude commands (roll, pitch,
             yawrate, and thrust) via the differential flatness properties of the quadcopter's
             dynamics.

      7. Execution
           - Finally, if the user has enabled it and tracking is available, the simulated
             experiment is executed on the Robotarium.

           - This stage handles take off, recording the required hovering thrust for the
             crazyflie, "play back" of the simulated trajectory, and data logging.

           - The resulting data is saved in a .pckl file for subsequent examination


  ros_objects/crazyflie_obj.py
  ----------------------------
    Facilitates the access to tracking data and the issuing of crazyflie commands through ROS.


  ros_objects/hoop_obj.py
  -----------------------
    Facilitates the access to tracking data and generating "helper" points (front, rear, avoid)
    for hoops in the Robotarium.


  sim_plotting/arena_plot.py
  --------------------------
    Plots the arena limits for the visualized simulation.


  sim_plotting/crazyflie_plot.py
  ------------------------------
    Plots and updates the location of the crazyflie for the visualized simulation.


  sim_plotting/hoop_plot.py
  -------------------------
    Plots the hoops and their "helper" points for the visualized simulation.


  trajectory/attitude_ctrl.py
  ---------------------------
    Includes a single function for converting from the differentially flat outputs (x, y, z,
    yaw) and their derivatives to attitude commands (roll, pitch, yawrate, thrust) which are
    what we need to actually control the crazyflie.


  trajectory/bezier_interp.py
  ---------------------------
    Includes two functions: one for producing the coefficients of a Bezier curve between two
    waypoints (for a specified segment time), the other for extracting the nominal state at a
    given time from these Bezier coefficients. 


  trajectory/path_planner.py
  --------------------------
    Contains functions which support the generation of the crazyflie's path. Waypoints are
    created, based on the input objectives, and will yield the desired behavior when
    interpolated between. Currently, the generated waypoints are simply x, y, z coordinates
    and a signed speed along the x-direction. Obstacle avoidance and automatic calculation of
    segment times (to achieve a prescribed average speed) are included.


  trajectory/time_dialation.py
  ----------------------------
    Includes a single function which prevents the desired state from getting too far ahead of
    the actual state. The desired state will progress based on how far behind the crazyflie is.


