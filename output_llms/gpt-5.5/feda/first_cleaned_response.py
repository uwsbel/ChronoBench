import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr








veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")






contact_method = chrono.ChContactMethod_NSC
tire_model = veh.TireModelType_TMEASY

init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)

chassis_fixed = False


vis_type = veh.VisualizationType_MESH


terrain_length = 200.0
terrain_width = 200.0
terrain_friction = 0.9
terrain_restitution = 0.01
terrain_texture = veh.GetDataFile("terrain/textures/tile4.jpg")


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3


render_fps = 50
render_step = 1.0 / render_fps
step_size = 1.0e-3
tire_step_size = step_size

render_steps = int(math.ceil(render_step / step_size))






feda = veh.FEDA()

feda.SetContactMethod(contact_method)
feda.SetChassisFixed(chassis_fixed)
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
feda.SetTireType(tire_model)
feda.SetTireStepSize(tire_step_size)

feda.Initialize()


feda.SetChassisVisualizationType(vis_type)
feda.SetSuspensionVisualizationType(vis_type)
feda.SetSteeringVisualizationType(vis_type)
feda.SetWheelVisualizationType(vis_type)
feda.SetTireVisualizationType(vis_type)

vehicle = feda.GetVehicle()
system = vehicle.GetSystem()
vehicle.EnableRealtime(True)






terrain = veh.RigidTerrain(system)

terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(terrain_friction)
terrain_mat.SetRestitution(terrain_restitution)

terrain_pose = chrono.ChCoordsysd(
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
)

patch = terrain.AddPatch(
    terrain_mat,
    terrain_pose,
    terrain_length,
    terrain_width
)

patch.SetTexture(terrain_texture, 80.0, 80.0)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono FEDA on Rigid Terrain")
vis.SetWindowSize(1280, 720)


track_point = chrono.ChVector3d(0.0, 0.0, 1.5)
chase_distance = 6.0
chase_height = 1.0
vis.SetChaseCamera(track_point, chase_distance, chase_height)

vis.AttachVehicle(vehicle)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()






driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step / steering_time)
driver.SetThrottleDelta(render_step / throttle_time)
driver.SetBrakingDelta(render_step / braking_time)

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
    feda.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    feda.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1