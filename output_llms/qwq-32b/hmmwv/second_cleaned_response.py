import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 200.0  
terrainWidth = 100.0


contact_method = chrono.ChContactMethod_NSC  


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


path_radius = 20.0
path = chrono.ChLineCircle()
path.SetRadius(path_radius)
path.SetPlane(chrono.ChPlane(chrono.ChVectorD(0, 0, terrainHeight), chrono.ChVectorD(0, 0, 1)))


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetThrottle(0.3)  
driver.SetSteeringPID(1.0, 0.0, 0.1)  


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


device = vis.GetDevice()
smgr = device.getSceneManager()
sentinel_node = smgr.addSphereSceneNode(0.5)
sentinel_node.setMaterialColor(chrono.ChColor(1, 0, 0))
sentinel_node.setMaterialFlag(chrono.EMF_LIGHTING, False)
sentinel_node.setPosition(chrono.ChVectorD(0, 0, terrainHeight))

target_node = smgr.addSphereSceneNode(0.5)
target_node.setMaterialColor(chrono.ChColor(0, 1, 0))
target_node.setMaterialFlag(chrono.EMF_LIGHTING, False)
target_node.setPosition(chrono.ChVectorD(0, 0, terrainHeight))


driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        
        sentinel_point = driver.GetSentinelPoint()
        target_point = driver.GetTargetPoint()
        sentinel_node.setPosition(chrono.ChVectorD(sentinel_point.x, sentinel_point.y, sentinel_point.z))
        target_node.setPosition(chrono.ChVectorD(target_point.x, target_point.y, target_point.z))
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 200.0  
terrainWidth = 100.0


contact_method = chrono.ChContactMethod_NSC  


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


path_radius = 20.0
path = chrono.ChLineCircle()
path.SetRadius(path_radius)
path.SetPlane(chrono.ChPlane(chrono.ChVectorD(0, 0, terrainHeight), chrono.ChVectorD(0, 0, 1)))


driver = veh.ChPathFollowerDriver(vehicle.GetVehicle(), path)
driver.SetThrottle(0.3)  
driver.SetSteeringPID(1.0, 0.0, 0.1)  


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Circular Path Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(-3.0, 0.0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


device = vis.GetDevice()
smgr = device.getSceneManager()
sentinel_node = smgr.addSphereSceneNode(0.5)
sentinel_node.setMaterialColor(chrono.ChColor(1, 0, 0))
sentinel_node.setMaterialFlag(chrono.EMF_LIGHTING, False)
sentinel_node.setPosition(chrono.ChVectorD(0, 0, terrainHeight))

target_node = smgr.addSphereSceneNode(0.5)
target_node.setMaterialColor(chrono.ChColor(0, 1, 0))
target_node.setMaterialFlag(chrono.EMF_LIGHTING, False)
target_node.setPosition(chrono.ChVectorD(0, 0, terrainHeight))


driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        
        sentinel_point = driver.GetSentinelPoint()
        target_point = driver.GetTargetPoint()
        sentinel_node.setPosition(chrono.ChVectorD(sentinel_point.x, sentinel_point.y, sentinel_point.z))
        target_node.setPosition(chrono.ChVectorD(target_point.x, target_point.y, target_point.z))
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)