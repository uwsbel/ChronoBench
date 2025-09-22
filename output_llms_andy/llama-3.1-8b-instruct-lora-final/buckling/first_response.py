import pychrono as ch
    import pychrono.fea as fea
    import pychrono.irrlicht as chrig
    import math

    # Initialize the Chrono physical system
    sys = ch.ChSystemNSC()

    # Define the contact material properties
    contact_mat = ch.ChContactMaterialNSC()
    contact_mat.SetFriction(0.5)
    contact_mat.SetRestitution(0.01)

    # Create a fixed truss body
    truss = fea.ChBodyEasyBox(1, 1, 1, 0.02, 7800, True, contact_mat)
    truss.SetPos(ch.ChVector3d(0, 0, 0))
    truss.SetFixed(True)
    sys.Add(truss)

    # Create a beam body
    beam = fea.ChBodyEasyBeam(1, 0.02, 0.02, 0.02, 7800, True, contact_mat)
    beam.SetPos(ch.ChVector3d(0.5, 0, 0))
    beam.SetRot(ch.QuatFromAngleAxis(.2, ch.ChVector3d(1, 0, 0)))
    sys.AddBody(beam)

    # Create a node body
    node = fea.ChBodyEasyNode()
    node.SetPos(ch.ChVector3d(-0.5, 0, 0))
    sys.AddBody(node)

    # Create a spring-damper constraint between the beam and the node
    constraint = fea.ChConstrainedMotionSDA()
    constraint.Initialize(beam, node, ch.ChVector3d(0, 0, 0), ch.ChVector3d(0, 0, 0))
    constraint.SetSpringCoefficient(100.0)
    constraint.SetDampingCoefficient(10.0)
    constraint.SetRestLength(0.1)
    constraint.SetUseDamping(True)
    sys.AddConstraint(constraint)

    # Create a motor constraint between the truss and the node
    motor = fea.ChMotorSDA()
    motor.Initialize(truss, node, ch.ChVector3d(0, 0, 0), ch.ChVector3d(0, 0, 0))
    motor.SetMotorFunction(lambda t: 0.1 * math.sin(10 * t))
    sys.AddConstraint(motor)

    # Create a visualization system
    vis = ch.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('FEA Beam')
    vis.Initialize()
    vis.AddLogo(ch.ChLogoType NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0.6, 0.3))
    vis.AddTypicalLights()

    # Set the solver type and timestepper
    ch.ChSolverPardiso().AsSolver().SetUseNotSparsityPattern(True)
    ch.ChChSystemTMEASRC().SetSolver(solver)
    ch.ChChSystemTMEASRC().SetTimestepper(ch.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

    # Run the simulation loop
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.001)