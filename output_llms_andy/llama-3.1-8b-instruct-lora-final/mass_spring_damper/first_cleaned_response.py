import pychrono as ch
    import pychrono.irrlicht as chrig
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, 0))
    
    ground = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    ground.SetPos(ch.ChVector3d(0, 0, 0))
    ground.SetFixed(True)
    sys.Add(ground)
    
    mass = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
    mass.SetPos(ch.ChVector3d(0, 0, 8))
    sys.Add(mass)
    
    spring_damper = ch.ChLinkTSDA()
    spring_damper.Initialize(mass, ground, ch.ChFRBodyBodyCenterBodyCenter())
    spring_damper.SetRestLength(7)
    spring_damper.SetDamping(0.2)
    spring_damper.SetSpringCoefficient(100)
    sys.Add(spring_damper)
    
    vis = ch.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('ChLinkTSDA demo')
    vis.Initialize()
    vis.AddLogo(ch.ChLogoType-logo_NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0, 6), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()
    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.001)