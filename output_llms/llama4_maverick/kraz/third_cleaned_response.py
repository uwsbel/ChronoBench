import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import numpy as np


initLocTruck = chrono.ChVector3d(-5, 0, 0.5)
initRotTruck = chrono.ChQuaterniond(1, 0, 0, 0)
initLocSedan = chrono.ChVector3d(0, -5, 0.5)
initRotSedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY


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
vehicleTruck.SetTireType(tire_model_truck)
vehicleTruck.Initialize()


vehicleSedan = veh.HMMWV_Full()
vehicleSedan.SetContactMethod(contact_method)
vehicleSedan.SetChassisCollisionType(chassis_collision_type)
vehicleSedan.SetChassisFixed(False)
vehicleSedan.SetInitPosition(chrono.ChCoordsysd(initLocSedan, initRotSedan))
vehicleSedan.SetTireType(tire_model_sedan)
vehicleSedan.Initialize()


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicleTruck.GetSystem())
mesh_file = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddMesh(patch_mat, mesh_file, 1.0, chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Multi-Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicleTruck.GetVehicle())


driverTruck = veh.ChInteractiveDriverIRR(vis)
driverSedan = veh.ChDriver(vehicleSedan.GetVehicle())


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3  
driverTruck.SetSteeringDelta(render_step_size / steering_time)
driverTruck.SetThrottleDelta(render_step_size / throttle_time)
driverTruck.SetBrakingDelta(render_step_size / braking_time)
driverTruck.Initialize()


driverSedan.SetSteering(0.0)
driverSedan.SetThrottle(0.5)
driverSedan.SetBraking(0.0)


states_truck = []


render_steps = int(render_step_size / step_size)


step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() :
    time = vehicleTruck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    driver_inputs_truck = driverTruck.GetInputs()
    driverSedan.Synchronize(time)
    driver_inputs_sedan = driverSedan.GetInputs()

    
    driverTruck.Synchronize(time)
    terrain.Synchronize(time)
    vehicleTruck.Synchronize(time, driver_inputs_truck, terrain)
    vehicleSedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    
    states_truck.append(vehicleTruck.GetVehicle().GetChassis().GetPos())

    
    driverTruck.Advance(step_size)
    driverSedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicleTruck.Advance(step_size)
    vehicleSedan.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)