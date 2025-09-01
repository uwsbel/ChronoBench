import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os

def main():
    # 0. Set the path to the Chrono data files (important for models and textures)
    chrono.SetChronoDataPath(veh.GetDataFile(""))

    # 1. Initialize the PyChrono environment and core components
    # Create a Chrono physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) # Gravity
    # Set solver settings if needed (default is usually fine for demos)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)


    # 2. Add the required physical systems and objects
    # Create the Kraz vehicle
    kraz_vehicle = veh.Kraz(system)

    # Set initial position and orientation of the vehicle
    # ChCoordsysD(ChVectorD position, ChQuaternion rotation)
    # Position: x, y (height), z
    # Rotation: Q_from_AngZ(angle_in_radians) for yaw
    initial_pos = chrono.ChVectorD(0, 0.6, 0) # Start slightly above ground
    initial_rot = chrono.Q_from_AngZ(0) # No initial yaw
    kraz_vehicle.SetInitPosition(chrono.ChCoordsysD(initial_pos, initial_rot))

    kraz_vehicle.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
    kraz_vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    kraz_vehicle.SetDriveType(veh.DrivelineTypeWV_AWD) # Wheeled Vehicle All-Wheel Drive
    kraz_vehicle.SetTireType(veh.TireModelType_TMEASY) # Using TMeasy tire model

    # Initialize the vehicle (this creates the subsystems)
    kraz_vehicle.Initialize()

    # Set visualization type for vehicle components
    kraz_vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    kraz_vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz_vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz_vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    for axle in kraz_vehicle.GetAxles():
        for wheel in axle.GetWheels():
            wheel.SetTireVisualizationType(veh.VisualizationType_MESH)


    # Create the rigid terrain
    terrain = veh.RigidTerrain(system)
    # Set contact material properties for the terrain
    # Parameters: friction_coefficient, restitution_coefficient, Young's modulus, Poisson's ratio
    # Note: Young's modulus and Poisson's ratio are less critical for NSC than for SMC
    contact_material = chrono.ChMaterialSurfaceNSC()
    contact_material.SetFriction(0.9)       # Coefficient of friction
    contact_material.SetRestitution(0.01)   # Coefficient of restitution
    terrain.SetContactMaterial(contact_material)

    # Initialize the terrain as a flat patch
    # Parameters: height, length_x, width_y
    terrain_height = 0.0
    terrain_length = 200.0 # meters
    terrain_width = 200.0  # meters
    terrain.Initialize(terrain_height, terrain_length, terrain_width)
    
    # Optionally, set a texture for the terrain
    texture_file = veh.GetDataFile("terrain/textures/tile4.jpg")
    if os.path.exists(texture_file):
        terrain.SetTexture(texture_file, 200, 200) # Texture file, length_repeat, width_repeat
    else:
        print(f"Warning: Terrain texture file not found: {texture_file}")


    # 3. Initialize a driver system for vehicle control
    # Create an Irrlicht-based GUI driver
    # This driver is controlled by keyboard inputs in the Irrlicht window
    # First, we need to create the Irrlicht visualization system

    # Create the Irrlicht visualization system
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Kraz Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(15, 10, 20), chrono.ChVectorD(0, 0, 0)) # Camera position and target
    vis.AddTypicalLights()
    
    # Attach the Chrono physical system to the Irrlicht visualization
    vis.AttachSystem(system) # This is crucial for Irrlicht to know what to render

    # Now create the driver system and link it to the vehicle and Irrlicht app
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.04)  # Steering sensitivity
    driver.SetThrottleDelta(0.04)  # Throttle sensitivity
    driver.SetBrakingDelta(0.1)    # Braking sensitivity
    driver.Initialize()


    # 4. Implement a simulation loop
    # Simulation parameters
    timestep = 0.005  # s (Corresponds to 200 Hz) - TMeasy tires may need smaller steps
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) # Good for vehicle dynamics

    # Real-time synchronization
    realtime_timer = chrono.ChRealtimeStepTimer()

    # Simulation loop
    while vis.Run():
        time = system.GetChTime()

        # Get driver inputs
        driver_inputs = driver.GetInputs()
        steering_input = driver_inputs.m_steering
        throttle_input = driver_inputs.m_throttle
        braking_input = driver_inputs.m_braking

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        kraz_vehicle.Synchronize(time, steering_input, braking_input, throttle_input, terrain)
        terrain.Synchronize(time) # Terrain might not do much in Synchronize for rigid terrain

        # Advance simulation for one timestep
        driver.Advance(timestep)
        kraz_vehicle.Advance(timestep)
        terrain.Advance(timestep)
        system.DoStepDynamics(timestep)

        # Update Irrlicht visualization
        vis.BeginScene()
        vis.Render()
        # Optional: Draw driver inputs on screen (HUD)
        driver.DrawAll(vis) # Renders the Irrlicht GUI elements for driver controls
        vis.EndScene()

        # Maintain real-time execution
        realtime_timer.Spin(timestep)

    print("Simulation ended.")

if __name__ == '__main__':
    main()