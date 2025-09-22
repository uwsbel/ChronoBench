import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





initLoc = chrono.ChVector3d(6, -70, 0.5)

initRot = chrono.Q_from_AngAxis(1.57, chrono.ChVector3d(0, 0, 1))


vis_type             = veh.VisualizationType_MESH
chassis_collision    = veh.CollisionType_NONE
tire_model           = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method    = chrono.ChContactMethod_NSC
step_size         = 1e-3
tire_step_size    = step_size
render_fps        = 50
render_step_size  = 1.0 / render_fps




vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())


highway_col = chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj')
patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    highway_col,
    add_collision=True,
    sweep_sphere_radius=0.01,
    smooth_mesh=False)


vis_mesh1 = chrono.ChTriangleMeshConnected()
vis_mesh1.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
mesh_shape1 = chrono.ChVisualShapeTriangleMesh()
mesh_shape1.SetMesh(vis_mesh1)
mesh_shape1.SetBackfaceCull(True)
patch1.GetGroundBody().AddVisualShape(mesh_shape1)





bump_col = veh.GetDataFile("terrain/meshes/bump.obj")
patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
    bump_col,
    add_collision=True,
    sweep_sphere_radius=0.01,
    smooth_mesh=False)


patch2.SetColor(chrono.ChColor(0.5, 0.5, 0.8))


dirt_tex = veh.GetDataFile("terrain/textures/dirt.jpg")
patch2.SetTexture(dirt_tex, 6.0, 6.0)


terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo – Extended Terrain')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())




driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()




print("VEHICLE MASS:", vehicle.GetVehicle().GetMass())

render_steps   = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    inputs = driver.GetInputs()

    
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, inputs, terrain)
    vis.Synchronize(t, inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)
    step_number += 1