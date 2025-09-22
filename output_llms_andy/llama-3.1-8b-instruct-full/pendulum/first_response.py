import pychrono as chrono
    import pychrono.irrlicht as chronoirr


    # Create the physical system
    sys = chrono.ChSystemNSC()

    # Create a ground body and add it to the simulation
    ground = chrono.ChBody()
    sys.Add(ground)
    ground.SetFixed(True)  # Fix the ground in place

    # Create a visualization cylinder for the ground
    cylinder_1 = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
    ground.AddVisualShape(cylinder_1, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

    # Create a pendulum body and add it to the simulation
    pend_1 = chrono.ChBody()
    sys.AddBody(pend_1)
    pend_1.SetPos(chrono.ChVector3d(0, 0, 1))  # Set the initial position of the pendulum
    pend_1.SetFixed(False)  # Allow the pendulum to move

    # Add mass and inertia to the pendulum
    pend_1.AddMass(1, chrono.ChVector3d(0, 0, 0))  # Add mass of 1 at the center of mass

    # Create a visualization cylinder for the pendulum
    cylinder_1 = chrono.ChVisualShapeCylinder(0.2, 2.5)  # Cylinder with radius 0.2 and height 2.5
    cylinder_1.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
    pend_1.AddVisualShape(cylinder_1)

    # Create a revolute joint to connect the pendulum to the ground
    rev_1 = chrono.ChLinkLockRevolute()
    rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
    sys.AddLink(rev_1)  # Add the revolute joint to the simulation

    # Create the Irrlicht visualization system
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)  # Attach the simulation system to the visualization
    vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
    vis.SetWindowTitle('ChBodyAuxRef demo')  # Set the window title
    vis.Initialize()  # Initialize the visualization system
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
    vis.AddSkyBox()  # Add a skybox for better visual appearance
    vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Add a camera to the visualization
    vis.AddTypicalLights()  # Add typical lights for better visualization

    # Simulation loop
    while vis.Run():
        vis.BeginScene()  # Begin the scene for rendering
        vis.Render()  # Render the scene
        vis.EndScene()  # End the scene

        sys.DoStepDynamics(1e-3)  # Perform a simulation step with a time step of 0.001 seconds

        # Log the pendulum's position and velocity
        pos_1 = pend_1.GetPos()
        print('t =', sys.GetChTime())
        print('  pos =', pos_1.x, pos_1.y)
        print('  vel =', pend_1.GetPosDt().x, pend_1.GetPosDt().y)