import pychrono as ch
    import pychrono.ros as chros
    import pychrono.irrlicht as chrig
    import math

    
    sys = ch.ChSystemNSC()
    sys.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, 0))

    
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, -0.5))
    ground.SetFixed(True)
    sys.Add(ground)

    
    rover = ch.ChBodyEasyBox(4, 2.5, 3, 1000)
    rover.SetPos(ch.ChVector3d(0, 0, 0.5))
    rover.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    rover.SetFixed(False)
    rover.SetUseTrullinertia(False)
    sys.AddBody(rover)

    
    rover_motor = ch.ChLinkMotorRotationSpeed()
    rover_motor.Initialize(rover, ground, ch.ChFramed(ch.ChVector3d(0, 0, 0.2), ch.ChVector3d(0, 0, -1)))
    rover_motor.SetMotorFunction(ch.ChFunctionConst(0))
    rover.SetMotorMotor(rover_motor)

    
    rover_steering = chros.ChInteractiveDriverROSPython()
    rover_steering.SetROSPythonHandler(rover, "curiosity/steering")
    rover_steering.Initialize(sys)

    
    vis = chrig.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 1024)
    vis.SetWindowTitle('Rigid rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(chrig.ChromeLogo())
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(11, 0, 4), ch.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(10, 10, 6), ch.ChVector3d(0, 0, 0.5), 10, 20, 20)
    vis.AddShadowMap()

    
    time_step = 1e-3

    
    time = 0
    time_end = 30
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        time += time_step
        sys.DoStepDynamics(time_step)
        if time < time_end:
            steering = math.pi / 2 * (math.cos(time * 0.1) + 1)
            rover_motor.SetMotorFunction(ch.ChFunctionConst(steering))