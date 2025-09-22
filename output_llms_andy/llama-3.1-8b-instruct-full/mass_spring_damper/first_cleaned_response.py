import pychrono.core as chrono
    import pychrono.irrlicht as chronoirr

    
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    
    ground = chrono.ChBody()
    sys.AddBody(ground)
    ground.SetFixed(True)
    ground.EnableCollision(False)

    
    ground_shape = chrono.ChVisualShapeTriangleMesh()
    mesh = chrono.ChTriangleMeshConnected()
    
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    ground_shape.SetMesh(mesh)
    ground_shape.SetMutable(False)
    ground.AddVisualShape(ground_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))

    
    mass = chrono.ChBody()
    sys.AddBody(mass)
    mass.SetPos(chrono.ChVector3d(0, -1, 0))
    mass.SetFixed(False)
    mass.EnableCollision(False)
    mass.SetMass(1)
    mass.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

    
    mass_shape = chrono.ChVisualShapeTriangleMesh()
    mesh = chrono.ChTriangleMeshConnected()
    
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_spring.obj"), False, True)
    mass_shape.SetMesh(mesh)
    mass_shape.SetMutable(False)
    mass.AddVisualShape(mass_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(1, 0, 0))))

    
    spring = chrono.ChLinkTSDA()
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    spring.SetRestLength(2)
    spring.SetSpringCoefficient(50)
    spring.SetDampingCoefficient(1)
    sys.AddLink(spring)

    
    damper = chrono.ChLinkTSDA()
    damper.Initialize(mass, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
    damper.SetRestLength(2)
    damper.SetSpringCoefficient(0)
    damper.SetDampingCoefficient(2)
    sys.AddLink(damper)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('ChLinkTSDA demo')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
    vis.AddTypicalLights()

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(1e-3)