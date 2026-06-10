import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


gator = veh.Gator()
gator.SetContactMethod(contact_method)
gator.SetChassisCollisionType(chassis_collision_type)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
gator.SetTireType(tire_model)
gator.SetTireStepSize(tire_step_size)

gator.Initialize()

gator.SetChassisVisualizationType(vis_type)
gator.SetSuspensionVisualizationType(vis_type)
gator.SetSteeringVisualizationType(vis_type)
gator.SetWheelVisualizationType(vis_type)
gator.SetTireVisualizationType(vis_type)

gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(gator.GetSystem())




patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 0), chrono.QUNIT),
    50, 50)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))





patch2 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, 25, 0), chrono.QUNIT),
    50, 50)


patch2.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch2.SetColor(chrono.ChColor(0.7, 0.7, 0.4))


ramp_mat = chrono.ChContactMaterialNSC()
ramp_mat.SetFriction(0.9)
ramp_mat.SetRestitution(0.01)


ramp = chrono.ChBody()
ramp.SetPos(chrono.ChVector3d(25, 25, 0))
ramp.SetRot(chrono.QUNIT)
ramp.SetMaterial(ramp_mat)
ramp.SetCollide(True)
ramp.SetBodyFixed(True)



ramp_seg1 = chrono.ChBody()
ramp_seg1.SetPos(chrono.ChVector3d(20, 25, 0.15))
ramp_seg1.SetRot(chrono.QUNIT)
ramp_seg1.SetMaterial(ramp_mat)
ramp_seg1.SetCollide(True)
ramp_seg1.SetBodyFixed(True)
ramp_seg1.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(3.0, 8.0, 0.15))
gator.GetSystem().AddBody(ramp_seg1)


ramp_seg2 = chrono.ChBody()
ramp_seg2.SetPos(chrono.ChVector3d(27, 25, 0.45))
ramp_seg2.SetRot(chrono.QUNIT)
ramp_seg2.SetMaterial(ramp_mat)
ramp_seg2.SetCollide(True)
ramp_seg2.SetBodyFixed(True)
ramp_seg2.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(3.0, 8.0, 0.45))
gator.GetSystem().AddBody(ramp_seg2)


ramp_seg3 = chrono.ChBody()
ramp_seg3.SetPos(chrono.ChVector3d(34, 25, 0.75))
ramp_seg3.SetRot(chrono.QUNIT)
ramp_seg3.SetMaterial(ramp_mat)
ramp_seg3.SetCollide(True)
ramp_seg3.SetBodyFixed(True)
ramp_seg3.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(3.0, 8.0, 0.75))
gator.GetSystem().AddBody(ramp_seg3)




patch3 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-25, -25, 0), chrono.QUNIT),
    50, 50)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch3.SetColor(chrono.ChColor(0.6, 0.6, 0.3))


bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.01)


bump1 = chrono.ChBody()
bump1.SetPos(chrono.ChVector3d(-15, -25, 0.15))
bump1.SetRot(chrono.QUNIT)
bump1.SetMaterial(bump_mat)
bump1.SetCollide(True)
bump1.SetBodyFixed(True)
bump1.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(1.5, 1.5, 0.15))
gator.GetSystem().AddBody(bump1)


bump2 = chrono.ChBody()
bump2.SetPos(chrono.ChVector3d(0, -25, 0.25))
bump2.SetRot(chrono.QUNIT)
bump2.SetMaterial(bump_mat)
bump2.SetCollide(True)
bump2.SetBodyFixed(True)
bump2.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(2.0, 2.0, 0.25))
gator.GetSystem().AddBody(bump2)


bump3 = chrono.ChBody()
bump3.SetPos(chrono.ChVector3d(15, -25, 0.1))
bump3.SetRot(chrono.QUNIT)
bump3.SetMaterial(bump_mat)
bump3.SetCollide(True)
bump3.SetBodyFixed(True)
bump3.SetCollisionShape(chrono.ChCollisionShape_BOX, chrono.ChVector3d(1.0, 1.0, 0.1))
gator.GetSystem().AddBody(bump3)




patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(25, -25, 0), chrono.QUNIT),
    50, 50)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 100)
patch4.SetColor(chrono.ChColor(0.5, 0.7, 0.3))

terrain.Initialize()






vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle - Multi-Terrain Test')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print("VEHICLE MASS:", gator.GetVehicle().GetMass())
print("Terrain configuration:")
print("  - Patch 1 (upper-left): Flat reference terrain")
print("  - Patch 2 (upper-right): Gradability test with ramp")
print("  - Patch 3 (lower-left): Terrain with bumps")
print("  - Patch 4 (lower-right): Grass terrain")


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = gator.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)