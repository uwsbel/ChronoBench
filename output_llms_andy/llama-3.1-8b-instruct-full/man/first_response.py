# Import necessary modules from the PyChrono library
    import pychrono.core as chrono
    import pychrono.irrlicht as irr
    import pychrono.vehicle as veh
    import math

    # Initialize the Chrono physical system with non-smooth contact (NSC) method
    sys = chrono.ChSystemNSC()

    # Add a ground body to the system, which will act as the terrain
    ground = veh.ChGround()
    ground.SetFixed(True)  # Fix the ground in place
    sys.Add(ground)  # Add the ground to the physical system

    # Create a MAN 10t truck vehicle and add it to the simulation
    truck = veh.ChTruckMAN10t()
    truck.SetContactMethod(sys.GetContactMethod())  # Set the contact method for the truck
    sys.Add(truck)  # Add the truck to the physical system

    # Initialize the truck, setting its initial position and orientation
    init_pos = chrono.ChVector3d(0, 0.5, 0)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    truck.Initialize(chrono.ChFramed(init_pos, init_rot))

    # Create a driver system for the truck
    driver = veh.ChInteractiveDriverIRR(truck.GetVehicle())
    sys.Add(driver)  # Add the driver to the physical system

    # Set the time response for steering and throttle keyboard inputs
    driver.SetSteeringDelta(0.0025)
    driver.SetThrottleDelta(0.0025)

    # Create a real-time visualization of the system using Irrlicht
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('MAN 10t Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.75, 0.0), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(truck.GetVehicle())  # Attach the truck to the visualization

    # Set the simulation time step
    time_step = 1e-3

    # Simulation loop
    time = 0
    while vis.Run():
        # Set driver inputs based on real-time user inputs
        driver.SetSteering(vis.GetSteering())
        driver.SetThrottle(vis.GetThrottle())
        driver.SetBraking(vis.GetBraking())

        # Update the vehicle dynamics (process driver inputs from previous step)
        truck.GetVehicle().Update()

        # Render the scene with updated vehicle position
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance the simulation by one time step
        time += time_step

        # Spin in place for real time to catch up
        time_step.sleep()