import pychrono as ch
    import pychrono.robot as chrobot
    import pychrono.irrlicht as chrig
    import math

    # Initialize the Chrono physical system with non-smooth contact (NSC) method.
    sys = ch.ChSystemNSC()

    # Define the gravitational acceleration in the simulation.
    g = ch.ChVector3d(0, 0, 0)

    # Create a ground body and add it to the physical system.
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, -1))
    ground.SetFixed(True)
    sys.Add(ground)

    # Create a Turtlebot robot and add it to the physical system.
    robot = chrobot.TurtleBot()
    robot.SetContactMethod(ch.ChContactMethod_NSC)
    robot.SetGroundBody(ground)
    sys.Add(robot)

    # Initialize the Turtlebot to specific positions and orientations.
    init_pos = ch.ChVector3d(0, 0.2, 0)
    init_rot = ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0))
    robot.Initialize(chrono.ChFramed(init_pos, init_rot))

    # Define the simulation time step.
    time_step = 2e-3

    # Create the Irrlicht visualization system.
    vis = ch.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Turtlebot NSC demo')
    vis.Initialize()
    vis.AddLogo(ch.ChLogoType NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0.6, 0.3), ch.ChVector3d(0, 0, 0.2))
    vis.AddTypicalLights()

    # Set the simulation time step for the Turtlebot.
    time = 0
    time_left = 0
    time_right = 0.5

    # Simulation loop.
    while vis.Run():
        # Update the Turtlebot's motors.
        robot.Update()

        # Get the current robot state.
        state = robot.GetChState()

        # Get the left and right wheel angles.
        left_angle = state.GetMotorLeft().GetAngle()
        right_angle = state.GetMotorRight().GetAngle()

        # Print the wheel angles.
        print('time:', time, 'left:', left_angle, 'right:', right_angle)

        # Increment the simulation time.
        time += time_step

        # Check if the left motor should be turned on.
        if time > time_left:
            robot.GetDeviceMotorLeft().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))
            time_left += 0.5

        # Check if the right motor should be turned on.
        if time > time_right:
            robot.GetDeviceMotorRight().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))
            time_right += 0.5

        # Render the scene.
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance the simulation by one timestep.
        sys.DoStepDynamics(time_step)

robot.GetDeviceMotorLeft().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))
    robot.GetDeviceMotorRight().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))

















