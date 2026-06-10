import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





initLoc = chrono.ChVector3d(-10, -2, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


contact_method = chrono.ChContactMethod_NSC


step_size = 2e-3


render_fps = 50




hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)


hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))


hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)


hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(step_size)


hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)




terrain = veh.RigidTerrain(hmmwv.GetSystem())


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT),
    32, 20)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0.15), chrono.QUNIT),
    32, 30)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)
patch2.SetColor(chrono.ChColor(1.0, 1.0, 1.0))


patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"))
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6, 6)


patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 42, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/test64.bmp"),
    64, 64,       
    0.0, 3.0)     
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)


terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on Complex Rigid Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0   
throttle_time = 1.0   
braking_time = 0.3    
driver.SetSteeringDelta(render_fps and (step_size / steering_time))
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)

driver.Initialize()




render_steps = math.ceil((1.0 / render_fps) / step_size)
step_number = 0
render_frame = 0


hmmwv.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1