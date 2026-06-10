import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


try:
    import rclpy
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS2 (rclpy) not available. Running without ROS integration.")






step_size = 1e-3


t_end = 30.0


terrain_length = 100.0  
terrain_width = 100.0   
terrain_friction = 0.9
terrain_restitution = 0.01


init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





print("Creating HMMWV vehicle...")


hmmwv = veh.HMMWV_Full()


hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)


hmmwv.SetChassisFixed(False)


hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)


hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))


hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)


hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)


hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)


hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)


hmmwv.SetTireType(veh.TireModelType_TMEASY)


hmmwv.SetTireStepSize(step_size)


hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

print("HMMWV vehicle created and initialized.")





print("Creating terrain...")


vehicle_system = hmmwv.GetSystem()


terrain = veh.RigidTerrain(vehicle_system)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)


patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrain_length,
    terrain_width
)


patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


terrain.Initialize()

print("Terrain created and initialized.")





print("Creating driver system...")


driver = veh.ChDriver(hmmwv.GetVehicle())


driver_inputs = veh.DriverInputs()
driver_inputs.m_steering = 0.0
driver_inputs.m_throttle = 0.0
driver_inputs.m_braking = 0.0


driver.Initialize()

print("Driver system created and initialized.")





if ROS_AVAILABLE:
    print("Setting up ROS integration...")
    
    
    ros_manager = chros.ChROSPythonManager()
    
    
    ros_manager.RegisterHandler(
        chros.ChROSClockHandler()
    )
    
    
    
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(
            25,  
            driver,
            "~/input/driver_inputs"  
        )
    )
    
    
    
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(
            25,  
            hmmwv.GetChassisBody(),
            "~/output/vehicle/state"  
        )
    )
    
    
    ros_manager.Initialize()
    
    print("ROS integration initialized.")
else:
    ros_manager = None
    print("Skipping ROS integration (ROS not available).")





print("Starting simulation loop...")
print(f"Simulation will run for {t_end} seconds with timestep {step_size} s")


time = 0.0
step_number = 0



def get_driver_inputs(t):
    
    inputs = veh.DriverInputs()
    
    if t < 2.0:
        
        inputs.m_throttle = min(0.5, t * 0.25)
        inputs.m_steering = 0.0
        inputs.m_braking = 0.0
    elif t < 8.0:
        
        inputs.m_throttle = 0.5
        inputs.m_steering = 0.0
        inputs.m_braking = 0.0
    elif t < 12.0:
        
        inputs.m_throttle = 0.3
        inputs.m_steering = 0.3 * min(1.0, (t - 8.0) / 2.0)
        inputs.m_braking = 0.0
    elif t < 18.0:
        
        inputs.m_throttle = 0.4
        inputs.m_steering = 0.0
        inputs.m_braking = 0.0
    else:
        
        inputs.m_throttle = 0.0
        inputs.m_steering = 0.0
        inputs.m_braking = min(1.0, (t - 18.0) * 0.2)
    
    return inputs


try:
    while time < t_end:
        
        
        time = vehicle_system.GetChTime()
        
        
        
        
        
        
        if ros_manager is not None:
            
            current_driver_inputs = driver.GetInputs()
        else:
            
            current_driver_inputs = get_driver_inputs(time)
        
        
        terrain.Synchronize(time)
        
        
        hmmwv.Synchronize(time, current_driver_inputs, terrain)
        
        
        driver.Synchronize(time)
        
        
        
        
        
        
        terrain.Advance(step_size)
        
        
        hmmwv.Advance(step_size)
        
        
        driver.Advance(step_size)
        
        
        
        
        
        if ros_manager is not None:
            
            if not ros_manager.Update(time, step_size):
                print("ROS manager update failed. Stopping simulation.")
                break
        
        
        
        
        
        
        if step_number % int(1.0 / step_size) == 0:
            
            pos = hmmwv.GetVehicle().GetPos()
            speed = hmmwv.GetVehicle().GetSpeed()
            
            print(f"Time: {time:.2f}s | "
                  f"Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | "
                  f"Speed: {speed:.2f} m/s | "
                  f"Throttle: {current_driver_inputs.m_throttle:.2f} | "
                  f"Steering: {current_driver_inputs.m_steering:.2f} | "
                  f"Braking: {current_driver_inputs.m_braking:.2f}")
        
        
        step_number += 1

except KeyboardInterrupt:
    print("\nSimulation interrupted by user.")





print("\nSimulation completed.")
print(f"Total simulation time: {time:.3f} seconds")
print(f"Total steps executed: {step_number}")


final_pos = hmmwv.GetVehicle().GetPos()
final_speed = hmmwv.GetVehicle().GetSpeed()
print(f"Final position: ({final_pos.x:.3f}, {final_pos.y:.3f}, {final_pos.z:.3f})")
print(f"Final speed: {final_speed:.3f} m/s")