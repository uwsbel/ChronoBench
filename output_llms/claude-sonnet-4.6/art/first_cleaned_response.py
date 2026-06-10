import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math





step_size = 1e-3


t_end = 1000.0


render_fps = 50
render_step_size = 1.0 / render_fps





init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)




terrain_length = 200.0   
terrain_width  = 100.0   
terrain_height = 0.0     





vehicle = veh.ARTcar()


vehicle.SetContactMethod(chrono.ChContactMethod_NSC)


vehicle.SetChassisCollisionType(veh.CollisionType_NONE)


vehicle.SetChassisFixed(False)


vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


vehicle.Initialize()


sys = vehicle.GetSystem()


print("Vehicle mass:           ", vehicle.GetVehicle().GetMass())




terrain = veh.RigidTerrain(sys)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QUNIT),
    terrain_length,
    terrain_width
)


patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


terrain.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("ARTcar - Rigid Terrain Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.SetHUDLocation(500, 20)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0    
throttle_time = 1.0    
braking_time  = 0.3    

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)


driver.Initialize()




print("=============================================================================")
print("ARTcar Simulation - Rigid Terrain")
print("=============================================================================")
print("Use arrow keys to control the vehicle:")
print("  UP    - Throttle")
print("  DOWN  - Brake")
print("  LEFT  - Steer Left")
print("  RIGHT - Steer Right")
print("=============================================================================")


step_number    = 0
render_steps   = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()


sim_time = 0.0

while vis.Run():
    sim_time = vehicle.GetSystem().GetChTime()

    
    if sim_time >= t_end:
        break

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    vehicle.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

print("\nSimulation ended at t = ", sim_time, "s")