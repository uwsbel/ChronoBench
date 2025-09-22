import pychrono as ch
    import pychrono.robot as chrobot
    import pychrono.irrlicht as chrig
    import math

    
    sys = ch.ChSystemNSC()

    
    g = ch.ChVector3d(0, 0, 0)

    
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, -1))
    ground.SetFixed(True)
    sys.Add(ground)

    
    robot = chrobot.TurtleBot()
    robot.SetContactMethod(ch.ChContactMethod_NSC)
    robot.SetGroundBody(ground)
    sys.Add(robot)

    
    init_pos = ch.ChVector3d(0, 0.2, 0)
    init_rot = ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0))
    robot.Initialize(chrono.ChFramed(init_pos, init_rot))

    
    time_step = 2e-3

    
    vis = ch.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Turtlebot NSC demo')
    vis.Initialize()
    vis.AddLogo(ch.ChLogoType NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0.6, 0.3), ch.ChVector3d(0, 0, 0.2))
    vis.AddTypicalLights()

    
    time = 0
    time_left = 0
    time_right = 0.5

    
    while vis.Run():
        
        robot.Update()

        
        state = robot.GetChState()

        
        left_angle = state.GetMotorLeft().GetAngle()
        right_angle = state.GetMotorRight().GetAngle()

        
        print('time:', time, 'left:', left_angle, 'right:', right_angle)

        
        time += time_step

        
        if time > time_left:
            robot.GetDeviceMotorLeft().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))
            time_left += 0.5

        
        if time > time_right:
            robot.GetDeviceMotorRight().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))
            time_right += 0.5

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        sys.DoStepDynamics(time_step)

robot.GetDeviceMotorLeft().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))
    robot.GetDeviceMotorRight().SetMotorFunction(ch.ChFunctionConst(chrono.CH_PI))