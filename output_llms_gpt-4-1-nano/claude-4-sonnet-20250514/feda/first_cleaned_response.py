import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

def main():
    
    print("Copyright (c) 2017 projectchrono.org")
    print("Chrono version: ", chrono.CHRONO_VERSION)
    
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    
    
    
    
    initLoc = chrono.ChVectorD(0, 0, 0.5)  
    initRot = chrono.ChQuaternionD(1, 0, 0, 0)  
    
    
    vehicle = veh.FEDA()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)  
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
    vehicle.SetTireType(veh.TireModelType_RIGID)  
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    
    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    
    terrain = veh.RigidTerrain(system)
    
    
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    
    patch = terrain.AddPatch(patch_mat, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                            chrono.QUNIT),
                           200, 200)  
    
    
    patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    
    terrain.Initialize()
    
    
    driver = veh.ChInteractiveDriverIRR()
    
    
    steering_time = 1.0  
    throttle_time = 1.0  
    braking_time = 0.3   
    
    driver.SetSteeringDelta(0.04)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    
    driver.Initialize()
    
    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('FEDA Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, -8, 2), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    
    
    camera_pos = chrono.ChVectorD(-8, 0, 2)  
    camera_target = chrono.ChVectorD(0, 0, 1)  
    
    
    step_size = 1e-3  
    tire_step_size = step_size  
    render_step_size = 1.0 / 50  
    
    
    time = 0
    render_time = 0
    
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        current_time = system.GetChTime()
        
        
        if (current_time >= render_time):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_time += render_step_size
            
            
            vehicle_pos = vehicle.GetPos()
            vehicle_rot = vehicle.GetRot()
            
            
            camera_world_pos = vehicle_pos + vehicle_rot.Rotate(camera_pos)
            camera_world_target = vehicle_pos + vehicle_rot.Rotate(camera_target)
            
            
            vis.GetActiveCamera().setPosition(chronoirr.vector3df(
                camera_world_pos.x, camera_world_pos.y, camera_world_pos.z))
            vis.GetActiveCamera().setTarget(chronoirr.vector3df(
                camera_world_target.x, camera_world_target.y, camera_world_target.z))
        
        
        driver_inputs = driver.GetInputs()
        
        
        steering_input = 0
        throttle_input = 0
        braking_input = 0
        
        if driver.GetInputMode() == veh.ChInteractiveDriverIRR.InputMode_KEYBOARD:
            
            pass  
        
        
        driver.Synchronize(current_time)
        terrain.Synchronize(current_time)
        vehicle.Synchronize(current_time, driver_inputs, terrain)
        
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        
        time += step_size
        
        
        realtime_timer.Spin(step_size)
        
        
        if math.fmod(current_time, 1.0) < step_size:
            vehicle_speed = vehicle.GetSpeed()
            print(f"Time: {current_time:.2f}s, Speed: {vehicle_speed:.2f} m/s")

    return 0

if __name__ == '__main__':
    main()