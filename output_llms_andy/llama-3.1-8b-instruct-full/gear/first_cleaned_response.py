import pychrono as chrono
    import pychrono.irrlicht as chronoirr
    import math

    
    sys = chrono.ChSystemNSC()

    
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(0.2)
    mat.SetRestitution(0.01)

    
    ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True, mat)
    ground.SetPos(chrono.ChVector3d(0, 0, -1))  
    ground.SetFixed(True)  
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(ground)

    
    bar = chrono.ChBodyEasyBox(0.2, 2, 0.2, 1000, True, True, mat)
    bar.SetPos(chrono.ChVector3d(1, 0, 0))  
    bar.SetRot(chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))  
    bar.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/metal.jpg"))
    sys.Add(bar)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(ground, bar, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    motor.SetSpeedFunction(chrono.ChFunctionConst(0.2))  
    sys.AddMotor(motor)

    
    gear1 = chrono.ChBodyEasyCylinder(0.4, 0.2, 100, True, True, mat)
    gear1.SetPos(chrono.ChVector3d(1.5, 0, 0))  
    gear1.SetRot(chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  
    gear1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/gear.jpg"))
    sys.Add(gear1)

    
    joint1 = chrono.ChLinkLockRevolute()
    joint1.Initialize(bar, gear1, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    sys.AddLink(joint1)

    
    gear2 = chrono.ChBodyEasyCylinder(0.3, 0.3, 100, True, True, mat)
    gear2.SetPos(chrono.ChVector3d(3, 0, 0))  
    gear2.SetRot(chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  
    gear2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/gear.jpg"))
    sys.Add(gear2)

    
    joint2 = chrono.ChLinkLockRevolute()
    joint2.Initialize(gear1, gear2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
    sys.AddLink(joint2)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  
    vis.SetWindowSize(1024, 768)  
    vis.SetWindowTitle('Gears and pulleys')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(
print("error happened with only start ```python")