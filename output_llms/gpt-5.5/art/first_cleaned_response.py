import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")






contact_method = chrono.ChContactMethod_NSC
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE


init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
init_pos = chrono.ChCoordsysd(init_loc, init_rot)


terrain_length = 200.0
terrain_width = 200.0
terrain_height = 0.0
terrain_friction = 0.9
terrain_restitution = 0.01


terrain_texture = veh.GetDataFile("terrain/textures/tile4.jpg")
terrain_texture_scale_x = 100
terrain_texture_scale_y = 100


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3


step_size = 1e-3
render_fps = 50
render_step_size = 1.0 / render_fps
render_steps = math.ceil(render_step_size / step_size)






vehicle = veh.ARTcar()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(init_pos)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

system = vehicle.GetSystem()






terrain = veh.RigidTerrain(system)

if contact_method == chrono.ChContactMethod_NSC:
    terrain_mat = chrono.ChContactMaterialNSC()
else:
    terrain_mat = chrono.ChContactMaterialSMC()

terrain_mat.SetFriction(terrain_friction)
terrain_mat.SetRestitution(terrain_restitution)

terrain_pose = chrono.ChCoordsysd(
    chrono.ChVector3d(0.0, 0.0, terrain_height),
    chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
)

patch = terrain.AddPatch(
    terrain_mat,
    terrain_pose,
    terrain_length,
    terrain_width
)

patch.SetTexture(
    terrain_texture,
    terrain_texture_scale_x,
    terrain_texture_scale_y
)

patch.SetColor(chrono.ChColor(0.55, 0.55, 0.50))

terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono ARTcar on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(
    chrono.ChVector3d(0.0, 0.0, 0.6),
    6.0,
    0.5
)

vis.Initialize()
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())






driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = system.GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1