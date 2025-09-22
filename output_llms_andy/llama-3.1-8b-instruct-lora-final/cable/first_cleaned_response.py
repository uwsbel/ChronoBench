import pychrono as ch
    import pychrono.irrlicht as chir
    import math

    
    sys = ch.ChSystemNSC()

    
    g = ch.ChVector3d(0, -9.81, 0)

    
    vis = ch.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('ChBoundedBeam ANCF Cable Elements')
    vis.Initialize()
    vis.AddLogo(chir.ChLogo())
    vis.AddSkyBox()
    vis.AddCamera(chir.ChCamera(chrono.Vector3df(2, 3, 6), chrono.Vector3df(0.5, 0.6, 1)))
    vis.AddTypicalLights()

    
    ground = ch.ChBodyEasyBox(6, 2, 6, 1000, False, False)
    ground.SetPos(ch.ChVector3d(0, 0, 0))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(ch.ChCoordsysColor(0.6, 0.3, 0.3))
    sys.Add(ground)

    
    mbeams = ch.ChBeamANCF()
    mbeams.SetSystem(sys)
    mbeams.SetNodes(20, 1)  
    mbeams.SetCrashTolerance(1e-4)
    mbeams.SetGravitationalAcceleration(g)
    mbeams.SetForceFunction(ch.ChFunctionTable().GetConstForce(100))
    mbeams.SetDrawNodes(True)
    mbeams.SetDrawBeam(True)

    
    for i in range(20):
        x = -5 + (i * 0.1)
        node = ch.ChNodeFEA()
        node.SetCoords(ch.ChVector3d(x, 0, 0))
        mbeams.AddNode(node)

    
    for i in range(19):
        mbeams.AddBeamElement(
            ch.ChBeamElementANCF(
                mbeams.GetNodes()[i],
                mbeams.GetNodes()[i + 1],
                0.1,  
                0.01  
            )
        )

    
    truss = ch.ChBody()
    truss.SetPos(ch.ChVector3d(0, 0, 0))
    truss.SetFixed(True)
    sys.Add(truss)

    
    constraint = ch.ChLinkMateGeneric()
    constraint.Initialize(mbeams.GetBody(), truss, False, False, False, False, False, False)
    constraint.SetConstrainedCoords(True, True, True, False, False, False, False, False, False)
    sys.Add(constraint)

    
    constraint = ch.ChLinkMateGeneric()
    constraint.Initialize(mbeams.GetBody(), ground, False, False, False, False, False, False)
    constraint.SetConstrainedCoords(True, True, True, False, False, False, False, False, False)
    sys.Add(constraint)

    
    box = ch.ChVisualShapeBox(1, 1, 1)
    truss.AddVisualShape(box)

    
    time_step = 0.001

    
    time = 0
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        sys.DoStepDynamics(time_step)

        
        mbeams.UpdateBeam()

        
        print("Nodal positions at t = {:.4f}".format(time))
        for i in range(20):
            print("  Node {0:2d} : {1}".format(i, mbeams.GetNodes()[i].GetCoords()))

        time += time_step