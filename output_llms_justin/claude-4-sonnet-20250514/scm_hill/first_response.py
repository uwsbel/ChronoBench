import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import numpy as np
import math

def main():
    print("Copyright (c) 2017 projectchrono.org")
    print("Chrono version: ", chrono.CHRONO_VERSION)
    
    # Initialize the PyChrono environment
    chrono.SetChronoDataPath("data/")
    veh.SetDataPath("data/vehicle/")
    
    # Create the HMMWV vehicle system
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.ChQuaternionD(1, 0, 0, 0)))
    vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    
    # Enable tire visualization
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # Set up the SCM deformable terrain
    terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
    
    # Set SCM soil parameters
    terrain.SetSoilParameters(
        0.2e6,   # Bekker Kphi parameter [Pa/m^n]
        0,       # Bekker Kc parameter [Pa/m^n]
        1.1,     # Bekker n exponent
        0,       # Cohesion [Pa]
        30,      # Friction angle [degrees]
        0.01,    # Flow number
        4e7,     # Elastic stiffness K [Pa/m]
        3e4      # Damping coefficient R [Pa*s/m]
    )
    
    # Configure bulldozing resistance
    terrain.SetBulldozingParameters(
        55,      # Angle of friction between soil and bulldozer blade [degrees]
        1,       # Ratio between bulldozing force and weight of soil bulldozed
        50,      # Erosion angle threshold [degrees]
        1.0,     # J_level (parameter for bulldozing force calculation)
        0.5      # K_level (parameter for bulldozing force calculation)
    )
    
    # Create a height map for the terrain (flat terrain with some variations)
    length = 20.0
    width = 20.0
    resolution = 0.02
    terrain.Initialize(length, width, resolution)
    
    # Set terrain patch properties
    terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 30000.2)
    
    # Initialize the driver system
    driver = veh.ChInteractiveDriverIRR()
    
    # Set the time response for steering and throttle inputs
    steering_time = 1.0  # Time to go from 0 to +1 (or -1)
    throttle_time = 1.0  # Time to go from 0 to +1
    braking_time = 0.3   # Time to go from 0 to +1
    
    driver.SetSteeringDelta(steering_time)
    driver.SetThrottleDelta(throttle_time)
    driver.SetBrakingDelta(braking_time)
    
    # Create the Irrlicht visualization system
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV SCM Deformable Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    
    # Attach visualization system to vehicle
    vis.AttachVehicle(vehicle.GetVehicle())
    
    # Initialize the driver with the visualization
    driver.Initialize()
    
    # Set simulation parameters
    step_size = 1e-3
    tire_step_size = step_size
    
    # Set real-time execution parameters
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    # Initialize simulation time
    time = 0
    end_time = 30.0
    
    # Simulation loop
    while vis.Run() and time < end_time:
        # Get current time
        time = vehicle.GetSystem().GetChTime()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
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
        
        # Render scene and output post-processing data
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)
        
        # Print vehicle mass (optional debug info)
        if math.fmod(time, 1.0) < step_size:
            print(f"Time: {time:.2f}s, Vehicle Mass: {vehicle.GetVehicle().GetMass():.2f} kg")

if __name__ == "__main__":
    main()