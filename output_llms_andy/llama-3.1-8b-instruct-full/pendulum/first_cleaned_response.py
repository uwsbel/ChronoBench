import pychrono as chrono
    import pychrono.irrlicht as chronoirr


    
    sys = chrono.ChSystemNSC()

    
    ground = chrono.ChBody()
    sys.Add(ground)
    ground.SetFixed(True)  

    
    cylinder_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  
    ground.AddVisualShape(cylinder_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

    
    pend_1 = chrono.ChBody()
    sys.AddBody(pend_1)
    pend_1.SetPos(chrono.ChVector3d(0, 0, 1))  
    pend_1.SetFixed(False)  

    
    pend_1.AddMass(1, chrono.ChVector3d(0, 0, 0))  

    
    cylinder_1 = chrono.ChVisualShapeCylinder(0.2, 2.5)  
    cylinder_1.SetColor(chrono.ChColor(0.6, 0, 0))  
    pend_1.AddVisualShape(cylinder_1)

    
    rev_1 = chrono.ChLinkLockRevolute()
    rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
    sys.AddLink(rev_1)  

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  
    vis.SetWindowSize(1024, 768)  
    vis.SetWindowTitle('ChBodyAuxRef demo')  
    vis.Initialize()  
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
    vis.AddSkyBox()  
    vis.AddCamera(chrono.ChVector3d(0, 3, 6))  
    vis.AddTypicalLights()  

    
    while vis.Run():
        vis.BeginScene()  
        vis.Render()  
        vis.EndScene()  

        sys.DoStepDynamics(1e-3)  

        
        pos_1 = pend_1.GetPos()
        print('t =', sys.GetChTime())
        print('  pos =', pos_1.x, pos_1.y)
        print('  vel =', pend_1.GetPosDt().x, pend_1.GetPosDt().y)