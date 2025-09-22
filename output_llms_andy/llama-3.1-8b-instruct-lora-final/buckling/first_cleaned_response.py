import pychrono as ch
    import pychrono.fea as fea
    import pychrono.irrlicht as chrig
    import math

    
    sys = ch.ChSystemNSC()

    
    contact_mat = ch.ChContactMaterialNSC()
    contact_mat.SetFriction(0.5)
    contact_mat.SetRestitution(0.01)

    
    truss = fea.ChBodyEasyBox(1, 1, 1, 0.02, 7800, True, contact_mat)
    truss.SetPos(ch.ChVector3d(0, 0, 0))
    truss.SetFixed(True)
    sys.Add(truss)

    
    beam = fea.ChBodyEasyBeam(1, 0.02, 0.02, 0.02, 7800, True, contact_mat)
    beam.SetPos(ch.ChVector3d(0.5, 0, 0))
    beam.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    sys.AddBody(beam)

    
    node = fea.ChBodyEasyNode()
    node.SetPos(ch.ChVector3d(-0.5, 0, 0))
    sys.AddBody(node)

    
    constraint = fea.ChConstrainedMotionSDA()
    constraint.Initialize(beam, node, ch.ChVector3d(0, 0, 0), ch.ChVector3d(0, 0, 0))
    constraint.SetSpringCoefficient(100.0)
    constraint.SetDampingCoefficient(10.0)
    constraint.SetRestLength(0.1)
    constraint.SetUseDamping(True)
    sys.AddConstraint(constraint)

    
    motor = fea.ChMotorSDA()
    motor.Initialize(truss, node, ch.ChVector3d(0, 0, 0), ch.ChVector3d(0, 0, 0))
    motor.SetMotorFunction(lambda t: 0.1 * math.sin(10 * t))
    sys.AddConstraint(motor)

    
    vis = ch.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('FEA Beam')
    vis.Initialize()
    vis.AddLogo(ch.ChLogoType NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0.6, 0.3))
    vis.AddTypicalLights()

    
    ch.ChSolverPardiso().AsSolver().SetUseNotSparsityPattern(True)
    ch.ChChSystemTMEASRC().SetSolver(solver)
    ch.ChChSystemTMEASRC().SetTimestepper(ch.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.001)