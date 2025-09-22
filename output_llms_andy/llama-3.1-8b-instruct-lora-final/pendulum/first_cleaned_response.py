import pychrono as ch
    import pychrono.irrlicht as chir

    
    sys = ch.ChSystemNSC()

    
    g = -10  

    
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(7, 0.3, 3, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, 0))
    ground.SetFixed(True)  
    sys.Add(ground)

    
    pend_mat = ch.ChContactMaterialNSC()
    pendulum = ch.ChBodyEasyCylinder(0.3, 1.0, 30, 1000, True, True, pend_mat)
    pendulum.SetPos(ch.ChVector3d(0, -0.5, 1))
    pendulum.SetRot(ch.QuatFromAngleAxis(.5, ch.ChVector3d(1, 0, 0)))
    sys.Add(pendulum)

    
    rev_1 = ch.ChLinkMateFix()
    rev_1.Initialize(ground, pendulum, False)
    sys.AddLink(rev_1)

    
    vis = chir.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('ChLinkTSD demo')
    vis.Initialize()
    vis.AddLogo(chir.ChLogoType-logo_NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0, 6), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    
    mvisualization_shape = ch.ChVisualShapeCylinder(0.3, 1.0)
    pendulum.AddVisualShape(mvisualization_shape, ch.ChFramed(pendulum.Frame(), ch.ChColor(0.6, 0, 0)))

    
    ch.ChSystem.GetChTime().SetTIC(time_step)
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        pos = pendulum.GetPos()
        lin_vel = pendulum.GetPosDt()
        print("Pendulum position: ", pos, "Velocity: ", lin_vel)

        
        sys.DoStepDynamics(time_step)

time_step = 2e-3  

sys.SetTimestep(time_step)  

while vis.Run():  
        sys.DoStepDynamics(time_step)  

vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  

print("Pendulum position: ", pos, "Velocity: ", lin_vel)  

sys.DoStepDynamics(time_step)