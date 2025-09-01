import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math






veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


chassis_collision_type = veh.CollisionType_NONE
suspension_collision_type = veh.CollisionType_PRIMITIVES
steering_collision_type = veh.CollisionType_PRIMITIVES
wheel_collision_type = veh.CollisionType_MESH
tire_collision_type = veh.CollisionType_MESH


terrain_collision_type = veh.CollisionType_MESH


terrainMesh = veh.GetDataFile('terrain/Highway/Highway_col.obj')


terrainMeshVis = veh.GetDataFile('terrain/Highway/Highway_vis.obj')


tire_model = veh.TireModelType_TMEASY
tire_step_size = 1e-3


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = 1e-3


render_step_size = 1.0 / 50  










hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.SetDriveline8WD(True)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(hmmwv.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
    chrono.CSYSNORM, 
    terrainMesh.GetLengthX(), terrainMesh.GetLengthZ(), 
    terrainMesh)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile(terrainMeshVis), 200, 200)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






print( "VEHICLE MASS: ",  hmmwv.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

hmmwv.GetVehicle().EnableRealtime(True)

while vis.Run() :
    time = hmmwv.GetSystem().GetChTime()

    
    if (time >= 10):
        break

    
    if (step_number % render_steps == 0) :
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