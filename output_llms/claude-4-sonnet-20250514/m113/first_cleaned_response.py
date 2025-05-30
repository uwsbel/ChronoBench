import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)
    
    
    vehicle = veh.M113()
    
    
    initLoc = chrono.ChVectorD(0, 0, 1.1)  
    initRot = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    
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
    
    
    vehicle.Initialize(system)
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)
    
    
    terrain = veh.RigidTerrain(system)
    
    
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.8)        
    patch_mat.SetRestitution(0.01)    
    patch = terrain.AddPatch(patch_mat, 
                           chrono.CSYSNORM,       
                           100.0, 100.0)         
    
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    
    driver = veh.ChInteractiveDriverIRR()
    
    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    
    driver.SetSteeringDelta(1.0 / 50)    
    driver.SetThrottleDelta(1.0 / 50)    
    driver.SetBrakingDelta(1.0 / 50)     
    
    
    driver.Initialize()
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('M113 Vehicle Simulation')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 8.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    
    
    vis.AttachVehicle(vehicle.GetVehicle())
    
    
    camera_pos = chrono.ChVectorD(-10, 0, 3)
    camera_target = chrono.ChVectorD(0, 0, 1)
    vis.GetSceneManager().getActiveCamera().setPosition(irr.vector3df(camera_pos.x, camera_pos.y, camera_pos.z))
    vis.GetSceneManager().getActiveCamera().setTarget(irr.vector3df(camera_target.x, camera_target.y, camera_target.z))
    
    
    step_size = 1e-3      
    realtime_timer = chrono.ChRealtimeStepTimer()
    time = 0
    
    
    while vis.Run():
        time = system.GetChTime()
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        driver_inputs = driver.GetInputs()
        
        
        throttle_input = driver_inputs.m_throttle
        steering_input = driver_inputs.m_steering
        braking_input = driver_inputs.m_braking
        
        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        vis.Advance(step_size)
        
        
        system.DoStepDynamics(step_size)
        
        
        realtime_timer.Spin(step_size)
        
        
        if int(time * 100) % 100 == 0:  
            vehicle_speed = vehicle.GetVehicle().GetSpeed()
            print(f"Time: {time:.2f}s, Speed: {vehicle_speed:.2f} m/s")

if __name__ == "__main__":
    main()