importchrono
importchrono.vehicle as vehicle
importchrono.sensors as sensors
importchrono.visualization as vis

def create_gator_simulation():
    # Initialize the Chrono system
    system = chrono.ChSystem()
    system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

    # Create the Gator vehicle
    gator = vehicle.Gator()
    gator.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 1), chrono.ChQuaternionD(1, 0, 0, 0)))

    # Configure vehicle visualization
    gator.SetChassisVisualizationType(vehicle.ChassisVisualizationType.TEXTURE)
    gator.SetWheelVisualizationType(vehicle.WheelVisualizationType.MESH)
    gator.SetSuspensionVisualizationType(vehicle.SuspensionVisualizationType.RIGIDLINKS)
    gator.SetSteeringVisualizationType(vehicle.SteeringVisualizationType.RIGIDLINKS)
    gator.SetTransmissionVisualizationType(vehicle.TransmissionVisualizationType.RIGIDLINKS)
    gator.SetDrivelineVisualizationType(vehicle.DrivelineVisualizationType.RIGIDLINKS)

    # Add rigid terrain
    terrain = chrono.ChTerrain(system)
    terrain.SetSize(100, 100)
    terrain.SetPosition(chrono.ChVectorD(0, 0, 0))
    terrain.SetTextureRepeat(20, 20)
    terrain.SetMaterial(chrono.ChMaterialSurface())
    terrain.SetCollisionMaterial(chrono.ChMaterialSurface())
    terrain.Build()

    # Create driver
    driver = chrono.ChDriver()
    driver.Attach(gator.GetChassis())
    driver.SetInputType(chrono.ChDriver.INPUT_KEYBOARD)

    # Create sensor manager
    sensor_manager = sensors.ChSensorManager(system)
    sensor_manager.SetRenderScene(True)

    # Add camera sensor
    camera = sensors.ChCameraSensor()
    camera.SetName("Main Camera")
    camera.SetPosition(chrono.ChVectorD(0, 1, 3))
    camera.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))
    camera.SetViewport(0, 0, 640, 480)
    camera.AttachTo(gator.GetChassis())
    sensor_manager.AddSensor(camera)

    # Add point lights
    light1 = sensors.ChPointLight()
    light1.SetName("Light 1")
    light1.SetPosition(chrono.ChVectorD(10, 10, 10))
    light1.SetIntensity(100)
    sensor_manager.AddSensor(light1)

    light2 = sensors.ChPointLight()
    light2.SetName("Light 2")
    light2.SetPosition(chrono.ChVectorD(-10, 10, 10))
    light2.SetIntensity(100)
    sensor_manager.AddSensor(light2)

    # Add visualization
    vis.addAllVisualizationAssets(gator)
    vis.addAllVisualizationAssets(terrain)

    # Simulation loop
    for step in range(1000):
        # Update driver
        driver.Update()

        # Update terrain
        terrain.Update()

        # Update vehicle
        gator.Update()

        # Update sensors
        sensor_manager.Update()

        # Render scene
        sensor_manager.Render()

        # Advance simulation
        system.DoStepDynamics(0.01)

        # Synchronize modules
        system.Synchronize()

    return system

# Run simulation
system = create_gator_simulation()