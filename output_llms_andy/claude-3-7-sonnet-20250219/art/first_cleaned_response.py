import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import numpy as np
import os


chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/"))





collision_method = chrono.ChCollisionSystemType_BULLET


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetCollisionSystemType(collision_method)





vehicle_pos = chrono.ChVectorD(0, 0.5, 0)
vehicle_rot = chrono.ChQuaternionD(1, 0, 0, 0)  


vehicle = veh.ARTCar(system)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(vehicle_pos, vehicle_rot))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(1e-3)
vehicle.SetInitFwdVel(0.0)
vehicle.Initialize()





terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


terrain_length = 200.0
terrain_width = 200.0
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(0, 0, 0),      
                         chrono.ChVectorD(0, 1, 0),      
                         terrain_length, terrain_width)   


texture_path = "terrain/textures/tile4.jpg"
patch.SetTexture(chrono.GetChronoDataFile(texture_path), 200, 200)


terrain.Initialize()





driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()





vis = chronoirr.ChIrrApp(system, "ARTCar Demo", chronoirr.dimension2du(1280, 720))
vis.AddTypicalLogo()
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddTypicalCamera(chronoirr.vector3df(0, 1.5, -8), chronoirr.vector3df(0, 0, 0))


vis.AssetBindAll()
vis.AssetUpdateAll()


driver.SetUserChIrrApp(vis)





step_size = 1e-3


render_step_size = 1.0 / 50.0





vis.SetTimestep(step_size)
vis.SetTryRealtime(True)


sim_time = 0.0
render_time = 0.0

print("Simulation started...")

while vis.GetDevice().run():
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(sim_time, driver_inputs, terrain)
    
    
    vehicle.Advance(step_size)
    
    
    terrain.Synchronize(sim_time)
    system.DoStepDynamics(step_size)
    
    
    sim_time += step_size
    
    
    render_time += step_size
    if render_time >= render_step_size:
        vis.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
        vis.DrawAll()
        vis.EndScene()
        render_time = 0
        
        
        print(f"Time: {sim_time:.2f}s | Speed: {vehicle.GetVehicle().GetVehicleSpeed():.2f} m/s | Steering: {driver_inputs.m_steering:.2f}")

print("Simulation ended.")