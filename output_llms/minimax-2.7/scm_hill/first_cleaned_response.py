import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np
import math
import os

def main():
    
    
    
    time_step = 0.001           
    end_time = 20.0             
    render_step_size = 1/30     
    
    
    
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    system.SetSolverType(chrono.ChSolver.Type_SOR)
    system.SetMaxItersSolverSpeed(50)
    system.SetMaxItersSolverStab(50)
    system.SetTol(1e-6)
    system.SetTolForce(1e-4)
    
    
    system.EnableMulticore(2)
    system.SetNumThreads(2)
    
    print("=" * 60)
    print("PyChrono HMMWV Simulation - Initialization Complete")
    print("=" * 60)

    
    
    

    
    
    
    print("\n[INFO] Setting up SCM Deformable Terrain...")
    
    
    terrain_height = 0.0        
    terrain_length = 200.0      
    terrain_width = 100.0      
    
    
    
    terrain = veh.SCMTerrain(system)
    
    
    terrain.Initialize(
        terrain_height,        
        terrain_length,        
        terrain_width,         
        200,                   
        100                    
    )
    
    
    
    
    print("[INFO] Configuring SCM soil parameters...")
    
    
    
    
    
    terrain.SetSoilParameters(
        0.2e6,                 
        0.1e6,                 
        0.01,                  
        1.1,                   
        0.3,                   
        1.0,                   
        5.0e4,                 
        6.0e6,                 
        4.0e4,                 
        2.0,                   
        0.5,                   
        0.03                   
    )
    
    
    terrain.SetLayerParameters(
        0.1,                   
        0.1e6,                 
        4.0e4,                 
        0.3,                   
        1.0,                   
        5.0e4,                 
        0.1                    
    )
    
    
    terrain.SetTexture(
        veh.GetDataFile("terrain/textures/grass.png"),  
        2.0,                   
        0.5                    
    )
    
    
    
    
    print("[INFO] Generating terrain height map...")
    
    
    def custom_height_map(x, y):
        
        
        height = 0.0
        
        
        height += 0.1 * math.sin(x * 0.05)
        height += 0.05 * math.sin(y * 0.08)
        
        
        height += 0.02 * math.sin(x * 0.3 + y * 0.2)
        height += 0.03 * math.cos(x * 0.15 - y * 0.25)
        
        
        height += x * 0.001
        
        
        if abs(x) < 5 and abs(y) < 5:
            height += 0.15 * math.exp(-(x**2 + y**2) / 10)
        
        return height
    
    
    terrain.Initialize(custom_height_map)
    
    
    
    
    print(f"[INFO] Terrain initialized: {terrain_length}m x {terrain_width}m")

    
    
    
    print("\n[INFO] Creating HMMWV vehicle...")
    
    
    vehicle_type = "HMMWV"              
    chassis_fixed = False               
    init_position = chrono.ChVectorD(0, 1.0, -3)  
    init_rotation = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    
    
    vehicle = veh.HMMWV(system)
    
    
    vehicle.SetChassisFixed(chassis_fixed)
    vehicle.SetChassisCollision(False)   
    vehicle.SetInitPosition(chrono.ChCoordsysD(init_position, init_rotation))
    
    
    
    vehicle.SetDrivelineType(veh.DrivelineType_AWD)
    
    
    vehicle.SetSuspensionType(veh.SuspensionType_DoubleWishbone)
    
    
    vehicle.Initialize()
    
    
    vehicle.SetSteeringVisualization(True)
    
    
    vehicle.SetWheelVisualization(veh.WheelVisualization_TYPE_CYLINDER)
    
    
    print(f"[INFO] Vehicle initialized: {vehicle_type}")
    print(f"       Chassis mass: {vehicle.GetChassisMass():.2f} kg")
    print(f"       Wheelbase: {vehicle.GetWheelbase():.2f} m")
    print(f"       Track width: {vehicle.GetTrackWidth():.2f} m")

    
    
    
    print("\n[INFO] Configuring vehicle control systems...")
    
    
    
    powertrain_config = veh.PowertrainConfig()
    powertrain_config.SetEngineType(veh.EngineModel_Type_SIMPLE)
    powertrain_config.SetEngineTorqueCurve(
        veh.ChBezierCurve([
            (0, 200), (500, 280), (1000, 350), (1500, 400),
            (2000, 420), (2500, 430), (3000, 420), (3500, 380),
            (4000, 320), (4500, 260), (5000, 200)
        ])
    )
    powertrain_config.SetEngineMaxTorque(450)  
    powertrain_config.SetEngineMaxPower(250000) 
    powertrain_config.SetEngineMaxRPM(4500)
    
    
    transmission_config = veh.TransmissionConfig()
    transmission_config.SetTransmissionType(veh.TransmissionModel_Type_AUTOMATIC)
    transmission_config.SetGearRatios([4.5, 3.0, 2.0, 1.5, 1.0, 0.8])
    transmission_config.SetShiftTorque(300)  
    
    
    powertrain = veh.Powertrain(system, powertrain_config, transmission_config)
    vehicle.InitializePowertrain(powertrain)
    
    
    
    for i in range(4):
        tire_config = veh.Pac89TireConfig()
        tire_config.SetLongitudinalStiffness(2.5e5)     
        tire_config.SetCorneringStiffness(8.0e4)         
        tire_config.SetMaximumSlip(0.15)                 
        tire_config.SetRelaxationLength(0.2)             
        vehicle.SetTireConfig(i, tire_config)

    
    
    

    
    
    
    print("\n[INFO] Setting initial conditions...")
    
    
    vehicle.SetVehicleVelocity(chrono.ChVectorD(0, 0, 0))
    
    
    for i in range(4):
        wheel_omega = 0.0  
        vehicle.SetWheelAngularVelocity(i, wheel_omega)
    
    
    steering_angle = 0.0  
    vehicle.SetSteering(steering_angle)
    
    
    
    
    print("[INFO] Configuring terrain-vehicle interaction...")
    
    
    for i in range(4):
        wheel = vehicle.GetWheel(i)
        
        
        wheel.GetWheelBody().SetMaterialFriction(0.7)
        
        
        contact_material = chrono.ChMaterialSurfaceNSC()
        contact_material.SetFriction(0.7)
        contact_material.SetRestitution(0.1)
        contact_material.SetCohesion(100)  
        
        
        wheel.GetWheelBody().SetCollide(True)
    
    
    
    
    print("\n[INFO] Initializing driver system...")
    
    
    
    driver = veh.ChInteractiveDriver(vehicle)
    
    
    driver.SetThrottleDelta(0.02)      
    driver.SetSteeringDelta(0.02)      
    driver.SetBrakingDelta(0.02)       
    
    
    
    class AutonomousDriver:
        
        def __init__(self, vehicle, target_speed=5.0):
            self.vehicle = vehicle
            self.target_speed = target_speed
            self.steering_controller = None
            self.speed_controller = None
            
        def Initialize(self):
            
            
            self.steering_controller = veh.PIDController(0.5, 0.0, 0.1)
            
            
            self.speed_controller = veh.PIDController(1.0, 0.1, 0.05)
            
        def GetSteering(self, time):
            
            
            radius = 50.0  
            angular_velocity = self.target_speed / radius
            
            
            steering = self.target_speed / (radius * 9.81 * 0.5)
            
            return np.clip(steering, -0.5, 0.5)
        
        def GetThrottle(self, time):
            
            current_speed = self.vehicle.GetVehicleSpeed()
            
            
            error = self.target_speed - current_speed
            throttle = np.clip(error * 0.1, 0, 1)
            
            return throttle
        
        def GetBraking(self, time):
            
            current_speed = self.vehicle.GetVehicleSpeed()
            
            
            if current_speed > self.target_speed * 1.1:
                return np.clip((current_speed - self.target_speed) * 0.1, 0, 1)
            return 0.0
    
    
    auto_driver = AutonomousDriver(vehicle, target_speed=5.0)
    auto_driver.Initialize()
    
    
    use_autonomous = False  
    
    if not use_autonomous:
        
        driver.Initialize()
        print("[INFO] Using interactive (keyboard) driver mode")
        print("       Controls: W=Throttle, S=Brake, A/D=Steering")
    else:
        print("[INFO] Using autonomous driver mode")

    
    
    

    print("\n[INFO] Setting up Irrlicht visualization...")

    
    vis = irr.ChIrrApp(
        system,                              
        "HMMWV Simulation",                  
        irr.dimension2du(1280, 720),         
        irr.EGUI_INVENTORY_SHOWING.NONE,     
        irr.EWORLD_FLAGS.WF_CONSTRAINT_WIREFRAME
    )
    
    
    vis.AddTypicalLights()
    
    
    vis.AddLightDirectional(
        irr.vector3df(1, -1, 1),             
        irr.SColorf(1.0, 0.98, 0.95),        
        3.0,                                 
        irr.vector3df(0, 0, 0)               
    )
    
    
    vis.AddSkyBox()
    
    
    
    
    print("[INFO] Configuring camera settings...")
    
    
    camera_pos = irr.vector3df(10, 5, 0)    
    camera_target = irr.vector3df(0, 0, 0)   
    
    vis.AddCamera(
        irr.vector3df(10, 5, -10),           
        irr.vector3df(0, 1, 0)               
    )
    
    
    vis.SetCameraTarget(irr.vector3df(0, 1, 0))
    vis.SetCameraPosition(irr.vector3df(15, 8, 0))
    
    
    vis.SetChaseCameraParameters(
        5.0,                                  
        2.0,                                  
        0.5,                                  
        0.01                                  
    )
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    coord_frame = irr.ChCoordinateFrame(system)
    coord_frame.Initialize(chrono.ChCoordsysD())
    vis.AddLinkageVisualization(coord_frame)
    
    
    vis.AddGrid(5.0, 20, chrono.ChColor(0.5, 0.5, 0.5))
    
    
    class SimulationHUD:
        
        def __init__(self, app):
            self.app = app
            self.font = None
            
        def Initialize(self):
            
            pass  
            
        def Update(self, time, vehicle, terrain):
            
            
            speed = vehicle.GetVehicleSpeed()
            pos = vehicle.GetVehiclePos()
            
            
            info_text = f"Time: {time:.2f}s | Speed: {speed:.2f} m/s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
            

    hud = SimulationHUD(vis)
    hud.Initialize()

    
    
    

    print("\n" + "=" * 60)
    print("SIMULATION STARTED")
    print("=" * 60)
    print(f"Duration: {end_time} seconds")
    print(f"Timestep: {time_step} seconds")
    print(f"Render FPS: {1/render_step_size:.0f}")
    print("-" * 60)

    
    simulation_time = 0.0
    render_time = 0.0
    step_number = 0
    
    
    vis.SetTimestep(time_step)
    vis.SetVideoframeSave(False)  
    vis.SetShowInfos(True)        
    
    
    vis.BeginScene()
    
    
    while simulation_time < end_time:
        
        
        
        if not use_autonomous:
            
            driver_inputs = driver.GetInputs()
        else:
            
            driver_inputs = veh.DriverInputs()
            driver_inputs.m_steering = auto_driver.GetSteering(simulation_time)
            driver_inputs.m_throttle = auto_driver.GetThrottle(simulation_time)
            driver_inputs.m_braking = auto_driver.GetBraking(simulation_time)
        
        
        
        
        
        vehicle.Synchronize(
            simulation_time,
            driver_inputs,
            terrain
        )
        
        
        terrain.Synchronize(simulation_time)
        
        
        driver.Synchronize(simulation_time)
        
        
        
        
        
        vehicle.Advance(time_step)
        
        
        terrain.Advance(time_step)
        
        
        system.DoStepDynamics(time_step)
        
        
        
        
        
        if simulation_time >= render_time:
            vis.BeginScene(True, True, irr.SColor(255, 140, 170, 200))
            vis.Render()
            
            
            hud.Update(simulation_time, vehicle, terrain)
            
            
            device = vis.GetDevice()
            if device:
                driver = device.getVideoDriver()
                if driver:
                    
                    info_text = f"Time: {simulation_time:.2f}s | Speed: {vehicle.GetVehicleSpeed():.2f} m/s"
                    
            
            vis.EndScene()
            render_time += render_step_size
        
        
        simulation_time += time_step
        step_number += 1
        
        
        if step_number % 500 == 0:
            print(f"[{simulation_time:.2f}s] Vehicle Position: ({vehicle.GetVehiclePos().x:.2f}, "
                  f"{vehicle.GetVehiclePos().y:.2f}, {vehicle.GetVehiclePos().z:.2f}) | "
                  f"Speed: {vehicle.GetVehicleSpeed():.2f} m/s")
    
    
    
    
    
    print("\n" + "=" * 60)
    print("SIMULATION COMPLETED")
    print("=" * 60)
    
    
    print(f"\nFinal Statistics:")
    print(f"  Total steps: {step_number}")
    print(f"  Final time: {simulation_time:.2f} seconds")
    print(f"  Final position: ({vehicle.GetVehiclePos().x:.3f}, "
          f"{vehicle.GetVehiclePos().y:.3f}, {vehicle.GetVehiclePos().z:.3f})")
    print(f"  Final speed: {vehicle.GetVehicleSpeed():.3f} m/s")
    
    
    if hasattr(terrain, 'GetMaxDepression'):
        print(f"  Max terrain depression: {terrain.GetMaxDepression():.3f} m")
    
    
    vis.Stop()  
    vehicle.Cleanup()  
    terrain.Cleanup()  
    
    print("\n[INFO] Simulation cleanup complete")
    print("=" * 60)





if __name__ == "__main__":
    print("PyChrono HMMWV Simulation Script")
    print("=" * 60)
    print("Initializing...")
    print()
    
    try:
        main()
    except Exception as e:
        print(f"\n[ERROR] Simulation failed with error: {e}")
        import traceback
        traceback.print_exc()