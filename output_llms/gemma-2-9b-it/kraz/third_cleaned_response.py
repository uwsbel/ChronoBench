import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLocTruck = chrono.ChVector3d(0, 0, 0.5)
initRotTruck = chrono.ChQuaterniond(1, 0, 0, 0)
initLocSedan = chrono.ChVector3d(10, 0, 0.5)
initRotSedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_modelTruck = veh.TireModelType_RIGID  
tire_modelSedan = veh.TireModelType_TMEASY


terrain_model = veh.RigidTerrain.MESH
terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicleTruck = veh.Kraz()
vehicleTruck.SetContactMethod(contact_method)
vehicleTruck.SetChassisCollisionType(chassis_collision_type)
vehicleTruck.SetChassisFixed(False)
vehicleTruck.SetInitPosition(chrono.ChCoordsysd(initLocTruck, initRotTruck))
vehicleTruck.Initialize()

vehicleTruck.SetChassisVisualizationType(vis_type, vis_type)
vehicleTruck.SetSteeringVisualizationType(vis_type)
vehicleTruck.SetSuspensionVisualizationType(vis_type, vis_type)
vehicleTruck.SetWheelVisualizationType(vis_type, vis_type)
vehicleTruck.SetTireVisualizationType(vis_type, vis_type)

vehicleTruck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicleSedan = veh.Sedan()
vehicleSedan.SetContactMethod(contact_method)
vehicleSedan.SetChassisCollisionType(chassis_collision_type)
vehicleSedan.SetChassisFixed(False)
vehicleSedan.SetInitPosition(chrono.ChCoordsysd(initLocSedan, initRotSedan))
vehicleSedan.Initialize()

vehicleSedan.SetChassisVisualizationType(vis_type, vis_type)
vehicleSedan.SetSteeringVisualizationType(vis_type)
vehicleSedan.SetSuspensionVisualizationType(vis_type, vis_type)
vehicleSedan.SetWheelVisualizationType(vis_type, vis_type)
vehicleSedan.SetTireVisualizationType(vis_type, vis_type)

vehicleSedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicleTruck.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)
patch.SetMeshFile(veh.GetDataFile('terrain/highway.obj'))  
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicleTruck.GetTractor())
vis.AttachVehicle(vehicleSedan.GetTractor())


driverTruck = veh.ChInteractiveDriverIRR(vis)
driverSedan = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driverTruck.SetSteeringDelta(render_step_size / steering_time)
driverTruck.SetThrottleDelta(render_step_size / throttle_time)
driverTruck.SetBrakingDelta(render_step_size / braking_time)

driverSedan.SetSteeringDelta(render_step_size / steering_time)
driverSedan.SetThrottleDelta(render_step_size / throttle_time)
driverSedan.SetBrakingDelta(render_step_size / braking_time)

driverSedan.SetThrottle(0.8)  
driverSedan.SetSteering(0)    

driverTruck.Initialize()
driverSedan.Initialize()


print( "VEHICLE MASS TRUCK: ",  vehicleTruck.GetTractor().GetMass())
print( "VEHICLE MASS SEDAN: ",  vehicleSedan.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicleTruck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driverTruck_inputs = driverTruck.GetInputs()
    driverSedan_inputs = driverSedan.GetInputs()

    
    driverTruck.Synchronize(time)
    terrain.Synchronize(time)
    vehicleTruck.Synchronize(time, driverTruck_inputs, terrain)
    vehicleSedan.Synchronize(time, driverSedan_inputs, terrain)
    vis.Synchronize(time, driverTruck_inputs, driverSedan_inputs)

    
    driverTruck.Advance(step_size)
    terrain.Advance(step_size)
    vehicleTruck.Advance(step_size)
    vehicleSedan.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)