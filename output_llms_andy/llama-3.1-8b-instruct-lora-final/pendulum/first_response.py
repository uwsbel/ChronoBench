import pychrono as ch
    import pychrono.irrlicht as chir

    # Initialize the Chrono physical system
    sys = ch.ChSystemNSC()

    # Define the gravitational acceleration in m/s
    g = -10  # m/s

    # Create a ground body and add it to the physical system
    ground_mat = ch.ChContactMaterialNSC()
    ground = ch.ChBodyEasyBox(7, 0.3, 3, 1000, True, True, ground_mat)
    ground.SetPos(ch.ChVector3d(0, 0, 0))
    ground.SetFixed(True)  # Fix the ground in place
    sys.Add(ground)

    # Create a pendulum body and add it to the physical system
    pend_mat = ch.ChContactMaterialNSC()
    pendulum = ch.ChBodyEasyCylinder(0.3, 1.0, 30, 1000, True, True, pend_mat)
    pendulum.SetPos(ch.ChVector3d(0, -0.5, 1))
    pendulum.SetRot(ch.QuatFromAngleAxis(.5, ch.ChVector3d(1, 0, 0)))
    sys.Add(pendulum)

    # Create a revolute joint to connect the pendulum to the ground
    rev_1 = ch.ChLinkMateFix()
    rev_1.Initialize(ground, pendulum, False)
    sys.AddLink(rev_1)

    # Create a visualization system
    vis = chir.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('ChLinkTSD demo')
    vis.Initialize()
    vis.AddLogo(chir.ChLogoType-logo_NONE)
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(0, 0, 6), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # Add visual shapes to the pendulum
    mvisualization_shape = ch.ChVisualShapeCylinder(0.3, 1.0)
    pendulum.AddVisualShape(mvisualization_shape, ch.ChFramed(pendulum.Frame(), ch.ChColor(0.6, 0, 0)))

    # Simulation loop
    ch.ChSystem.GetChTime().SetTIC(time_step)
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Log the pendulum's position and velocity
        pos = pendulum.GetPos()
        lin_vel = pendulum.GetPosDt()
        print("Pendulum position: ", pos, "Velocity: ", lin_vel)

        # Advance the simulation by one time step
        sys.DoStepDynamics(time_step)

time_step = 2e-3  # Set the time step for the simulation

sys.SetTimestep(time_step)  # Set the time step for the simulation system

while vis.Run():  # Main simulation loop
        sys.DoStepDynamics(time_step)  # Advance the simulation by one time step

vis.BeginScene()  # Begin rendering the scene
    vis.Render()  # Render the current frame
    vis.EndScene()  # End rendering the scene

print("Pendulum position: ", pos, "Velocity: ", lin_vel)  # Log the pendulum's position and velocity

sys.DoStepDynamics(time_step)  # Advance the simulation by one time step