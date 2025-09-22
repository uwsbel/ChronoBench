import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import numpy as np

def main():
    # Initialize PyChrono environment
    chrono.SetChronoDataPath("path/to/chrono/data/")  # Set your Chrono data path
    veh.SetDataPath("path/to/chrono/data/vehicle/")   # Set vehicle data path
    
    # Create the Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # Set solver parameters for better performance
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)
    
    # Create the terrain
    terrain = veh.RigidTerrain(system)
    
    # Create terrain patch with custom material properties
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat,
                           chrono.CSYSNORM,
                           200, 200)  # 200m x 200m terrain
    
    # Set terrain texture
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    # Create vehicle (BMW E90 Sedan approximation using available sedan model)
    # Vehicle initial position and orientation
    initLoc = chrono.ChVectorD(0, 0, 0.5)
    initRot = chrono.ChQuaternionD(1, 0, 0, 0)
    
    # Create the vehicle
    vehicle = veh.BMW_E90()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    
    # Enable visualization for vehicle parts
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # Configure TMEASY tire model parameters
    for axle in vehicle.GetAxles():
        for wheel in axle.GetWheels():
            tire = wheel.GetTire()
            if isinstance(tire, veh.TMeasyTire):
                # Set tire parameters for better realism
                tire.SetVerticalStiffness(165000.0)
                tire.SetRollingResistance(0.01)
                tire.SetLateralStiffness(180000.0)
                tire.SetLongitudinalStiffness(200000.0)
    
    # Create the driver system
    driver = veh.ChInteractiveDriverIRR()
    
    # Set the time response for steering and throttle inputs
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(1.0 / steering_time)
    driver.SetThrottleDelta(1.0 / throttle_time)
    driver.SetBrakingDelta(1.0 / braking_time)
    
    # Initialize the driver
    driver.Initialize()
    
    # Create the Irrlicht visualization system
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('BMW E90 Sedan - PyChrono Vehicle Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 0, 0.75), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    
    # Add additional directional lighting
    vis.AddLight(chrono.ChVectorD(30, 100, 30), chrono.ChColor(1, 1, 1), chrono.ChColor(1, 1, 1))
    vis.AddLight(chrono.ChVectorD(-30, 100, -30), chrono.ChColor(0.7, 0.7, 0.7), chrono.ChColor(0.7, 0.7, 0.7))
    
    # Attach visualization system to vehicle
    vis.AttachVehicle(vehicle.GetVehicle())
    
    # Add terrain logo/texture
    logo_asset = chrono.ChVisualShapeTexture()
    logo_asset.SetTextureFilename(chrono.GetChronoDataFile("logo_chronoengine_alpha.png"))
    
    # Create a visual shape for the terrain with logo
    terrain_shape = chrono.ChVisualShapeBox(chrono.ChVectorD(10, 10, 0.1))
    terrain_shape.SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    
    # Simulation parameters
    step_size = 1e-3
    tire_step_size = 1e-3
    render_step_size = 1.0 / 50  # FPS = 50
    
    # Initialize simulation time
    time = 0
    render_time = 0
    
    # Create data collection for analysis (optional)
    vehicle_pos = []
    vehicle_speed = []
    steering_input = []
    throttle_input = []
    
    print("Controls:")
    print("  W/S: Throttle/Brake")
    print("  A/D: Steering")
    print("  ESC: Exit simulation")
    
    # Simulation loop
    while vis.Run():
        current_time = system.GetChTime()
        
        # Render scene and output POV-Ray data
        if (current_time >= render_time):
            vis.BeginScene()
            vis.Render()
            
            # Display vehicle information
            speed = vehicle.GetSpeed()
            engine_rpm = vehicle.GetEngine().GetMotorSpeed() * 60 / (2 * math.pi)
            gear = vehicle.GetTransmission().GetCurrentGear()
            
            # Display information on screen
            info_text = f"Time: {current_time:.2f} s\n"
            info_text += f"Speed: {speed * 3.6:.1f} km/h\n"
            info_text += f"Engine RPM: {engine_rpm:.0f}\n"
            info_text += f"Gear: {gear}\n"
            info_text += f"Throttle: {driver.GetThrottle():.2f}\n"
            info_text += f"Steering: {driver.GetSteering():.2f}\n"
            info_text += f"Braking: {driver.GetBraking():.2f}"
            
            vis.WriteImageToFile(f"output_{int(current_time*100):06d}.jpg")
            vis.EndScene()
            render_time += render_step_size
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Driver system update
        driver.Synchronize(current_time)
        terrain.Synchronize(current_time)
        vehicle.Synchronize(current_time, driver_inputs, terrain)
        vis.Synchronize(current_time, driver_inputs)
        
        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Collect data for analysis
        pos = vehicle.GetPos()
        vehicle_pos.append([current_time, pos.x, pos.y, pos.z])
        vehicle_speed.append([current_time, speed])
        steering_input.append([current_time, driver.GetSteering()])
        throttle_input.append([current_time, driver.GetThrottle()])
        
        # Update simulation time
        time += step_size
        
        # Optional: Add some automatic scenario (uncomment to enable)
        # if current_time > 5.0 and current_time < 10.0:
        #     # Automatic steering maneuver for demonstration
        #     auto_steering = 0.3 * math.sin(2 * math.pi * (current_time - 5.0) / 5.0)
        #     driver.SetSteering(auto_steering)
        
        # Safety check - stop if vehicle goes too far
        vehicle_position = vehicle.GetPos()
        if abs(vehicle_position.x) > 100 or abs(vehicle_position.y) > 100:
            print("Vehicle went out of bounds. Stopping simulation.")
            break
    
    # Cleanup and save data
    print(f"\nSimulation completed. Total time: {time:.2f} seconds")
    print(f"Final vehicle position: ({vehicle_position.x:.2f}, {vehicle_position.y:.2f}, {vehicle_position.z:.2f})")
    
    # Optional: Save collected data to files
    try:
        np.savetxt("vehicle_position.csv", vehicle_pos, delimiter=",", 
                   header="Time,X,Y,Z", comments="")
        np.savetxt("vehicle_speed.csv", vehicle_speed, delimiter=",", 
                   header="Time,Speed", comments="")
        np.savetxt("steering_input.csv", steering_input, delimiter=",", 
                   header="Time,Steering", comments="")
        np.savetxt("throttle_input.csv", throttle_input, delimiter=",", 
                   header="Time,Throttle", comments="")
        print("Simulation data saved to CSV files.")
    except Exception as e:
        print(f"Could not save data: {e}")

# Additional utility functions for enhanced functionality
def create_custom_terrain_patch(terrain, system, position, size, texture_file=None):
    """Create a custom terrain patch with specified properties"""
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.8)
    patch_mat.SetRestitution(0.1)
    
    patch = terrain.AddPatch(patch_mat,
                           chrono.ChCoordsysD(position, chrono.QUNIT),
                           size[0], size[1])
    
    if texture_file:
        patch.SetTexture(texture_file, size[0], size[1])
    
    return patch

def add_terrain_obstacles(system, terrain):
    """Add some obstacles to the terrain for more interesting simulation"""
    # Create some barriers
    for i in range(5):
        barrier = chrono.ChBodyEasyBox(2, 0.5, 1, 1000, True, True)
        barrier.SetPos(chrono.ChVectorD(20 + i * 10, 10, 0.5))
        barrier.SetBodyFixed(True)
        barrier.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
        system.Add(barrier)

if __name__ == "__main__":
    main()