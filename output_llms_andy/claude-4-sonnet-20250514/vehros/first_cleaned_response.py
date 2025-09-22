import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.ros as chros
import math
import numpy as np

def main():
    
    print("Initializing PyChrono environment...")
    
    
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    system.SetContactForceModel(chrono.ChSystemNSC.ContactForceModel_Hertz)
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    
    
    initLoc = chrono.ChVectorD(0, 0, 1.6)
    initRot = chrono.ChQuaternionD(1, 0, 0, 0)
    
    
    print("Creating HMMWV vehicle...")
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    
    
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    
    
    tire_model = veh.TireModelType_TMEASY
    vehicle.SetTireType(tire_model)
    vehicle.SetTireStepSize(1e-3)
    
    
    vehicle.Initialize()
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh_VisualizationType_MESH)
    
    
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    
    print("Creating terrain...")
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    
    
    terrain_length = 200.0
    terrain_width = 200.0
    
    
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch_mat.SetPoissonRatio(0.3)
    
    patch = terrain.AddPatch(patch_mat, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                            chrono.QUNIT),
                           terrain_length, terrain_width)
    
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    
    print("Initializing driver system...")
    driver = veh.ChInteractiveDriverIRR()
    
    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    
    driver.Initialize()
    
    
    print("Initializing ROS integration...")
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    
    
    driver_inputs_topic = "~/input/driver_inputs"
    driver_inputs_handler = chros.ChROSDriverInputsHandler(25, driver_inputs_topic)
    ros_manager.RegisterHandler(driver_inputs_handler)
    
    
    vehicle_state_topic = "~/output/vehicle/state"
    vehicle_state_handler = chros.ChROSBodyHandler(25, vehicle_state_topic, vehicle.GetChassisBody())
    ros_manager.RegisterHandler(vehicle_state_handler)
    
    
    wheel_states_topic = "~/output/vehicle/wheel_states"
    wheel_handler = chros.ChROSVehicleHandler(25, wheel_states_topic, vehicle)
    ros_manager.RegisterHandler(wheel_handler)
    
    
    ros_manager.Initialize()
    
    
    print("Setting up visualization...")
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Simulation with ROS')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())
    
    
    step_size = 1e-3
    tire_step_size = step_size
    render_step_size = 1.0/50  
    
    
    time = 0
    render_steps = math.ceil(render_step_size / step_size)
    step_number = 0
    render_frame = 0
    
    
    print("Starting simulation loop...")
    
    while vis.Run():
        time = vehicle.GetSystem().GetChTime()
        
        
        if not ros_manager.Update(time, step_size):
            break
            
        
        driver_inputs = driver_inputs_handler.GetDriverInputs()
        
        
        if driver_inputs.m_steering != 0 or driver_inputs.m_throttle != 0 or driver_inputs.m_braking != 0:
            
            driver.SetSteering(driver_inputs.m_steering)
            driver.SetThrottle(driver_inputs.m_throttle)
            driver.SetBraking(driver_inputs.m_braking)
        else:
            
            driver.Synchronize(time)
        
        
        steering = driver.GetSteering()
        throttle = driver.GetThrottle()  
        braking = driver.GetBraking()
        
        
        terrain.Synchronize(time)
        vehicle.Synchronize(time, steering, throttle, braking, terrain)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        
        
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1
            
        
        if step_number % 1000 == 0:
            vehicle_pos = vehicle.GetPos()
            vehicle_speed = vehicle.GetSpeed()
            print(f"Time: {time:.2f}s, Position: ({vehicle_pos.x:.2f}, {vehicle_pos.y:.2f}, {vehicle_pos.z:.2f}), Speed: {vehicle_speed:.2f} m/s")
        
        step_number += 1
        
        
        if vehicle_pos.z < -5.0:  
            print("Vehicle fell below ground level. Stopping simulation.")
            break
            
        if time > 300:  
            print("Maximum simulation time reached.")
            break
    
    print("Simulation completed.")

if __name__ == '__main__':
    main()