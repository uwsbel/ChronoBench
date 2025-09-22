import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    # Set the data path for PyChrono vehicle models
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Initialize the PyChrono environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # Create the Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # Set solver parameters
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)
    
    # Create the M113 vehicle
    vehicle = veh.M113()
    
    # Set initial vehicle parameters
    initLoc = chrono.ChVectorD(0, 0, 1.1)  # Initial position (slightly above ground)
    initRot = chrono.ChQuaternionD(1, 0, 0, 0)  # Initial orientation (no rotation)
    
    # Initialize the vehicle at the specified position and orientation
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeTV_SIMPLE)
    vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
    vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
    vehicle.SetTireStepSize(1e-3)
    
    # Initialize the vehicle
    vehicle.Initialize(system)
    
    # Set visualization types for vehicle parts
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)
    
    # Create the rigid terrain
    terrain = veh.RigidTerrain(system)
    
    # Define terrain patch properties
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.8)        # Friction coefficient
    patch_mat.SetRestitution(0.01)    # Restitution coefficient
    patch = terrain.AddPatch(patch_mat, 
                           chrono.CSYSNORM,       # Coordinate system
                           100.0, 100.0)         # Dimensions (length x width)
    
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    # Create the driver system
    driver = veh.ChInteractiveDriverIRR()
    
    # Set the time response for steering and throttle inputs
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    
    driver.SetSteeringDelta(1.0 / 50)    # steering input increment
    driver.SetThrottleDelta(1.0 / 50)    # throttle input increment
    driver.SetBrakingDelta(1.0 / 50)     # braking input increment
    
    # Initialize the driver
    driver.Initialize()
    
    # Create the Irrlicht visualization system
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('M113 Vehicle Simulation')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 8.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    
    # Attach visualization system to vehicle
    vis.AttachVehicle(vehicle.GetVehicle())
    
    # Set camera initial position
    camera_pos = chrono.ChVectorD(-10, 0, 3)
    camera_target = chrono.ChVectorD(0, 0, 1)
    vis.GetSceneManager().getActiveCamera().setPosition(irr.vector3df(camera_pos.x, camera_pos.y, camera_pos.z))
    vis.GetSceneManager().getActiveCamera().setTarget(irr.vector3df(camera_target.x, camera_target.y, camera_target.z))
    
    # Simulation parameters
    step_size = 1e-3      # Integration step size
    realtime_timer = chrono.ChRealtimeStepTimer()
    time = 0
    
    # Simulation loop
    while vis.Run():
        time = system.GetChTime()
        
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Driver inputs
        throttle_input = driver_inputs.m_throttle
        steering_input = driver_inputs.m_steering
        braking_input = driver_inputs.m_braking
        
        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)
        
        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)
        
        # Advance the system dynamics
        system.DoStepDynamics(step_size)
        
        # Maintain real-time execution
        realtime_timer.Spin(step_size)
        
        # Optional: Print some vehicle information
        if int(time * 100) % 100 == 0:  # Print every second
            vehicle_speed = vehicle.GetVehicle().GetSpeed()
            print(f"Time: {time:.2f}s, Speed: {vehicle_speed:.2f} m/s")

if __name__ == "__main__":
    main()