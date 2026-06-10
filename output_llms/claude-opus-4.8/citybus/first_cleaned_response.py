import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_vis_type    = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type   = veh.VisualizationType_PRIMITIVES
wheel_vis_type      = veh.VisualizationType_MESH
tire_vis_type       = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0.0
terrainLength = 200.0   
terrainWidth  = 200.0   


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3


fps = 50
render_step_size = 1.0 / fps





bus = veh.CityBus()
bus.SetContactMethod(contact_method)
bus.SetChassisCollisionType(chassis_collision_type)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
bus.SetTireType(tire_model)
bus.SetTireStepSize(step_size)


bus.Initialize()


bus.SetChassisVisualizationType(chassis_vis_type)
bus.SetSuspensionVisualizationType(suspension_vis_type)
bus.SetSteeringVisualizationType(steering_vis_type)
bus.SetWheelVisualizationType(wheel_vis_type)
bus.SetTireVisualizationType(tire_vis_type)





terrain = veh.RigidTerrain(bus.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)


patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('CityBus on Rigid Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(bus.GetVehicle())





driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0    
throttle_time = 1.0    
braking_time  = 0.3    

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






render_steps = math.ceil(render_step_size / step_size)
step_number = 0
render_frame = 0

bus.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = bus.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    bus.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    bus.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1