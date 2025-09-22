import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLocTruck = chrono.ChVector3d(0, 0, 0.5)
initRotTruck = chrono.ChQuaterniond(1, 0, 0, 0)


initLocSedan = chrono.ChVector3d(0, 0, 0.5)
initRotSedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0      
terrainLength = 400.0  
terrainWidth = 20.0   


trackPointTruck = chrono.ChVector3d(0,0, 2.1)


trackPointSedan = chrono.ChVector3d(0,0, 2.1)


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


vehicleSedan = veh.Sedan()
vehicleSedan.SetContactMethod(contact_method)
vehicleSedan.SetChassisCollisionType(chassis_collision_type)
vehicleSedan.SetChassisFixed(False)
vehicleSedan.SetInitPosition(chrono.ChCoordsysd(initLocSedan, initRotSedan))
vehicleSedan.Initialize()


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicleTruck.GetSystem())
terrain.CreateBoxMesh(chrono.ChVector3d(0, 0, 0), terrainLength, terrainWidth, 1, 1)
terrain.Initialize()


visTruck = veh.ChWheeledVehicleVisualSystemIrrlicht()
visTruck.SetWindowTitle('Kraz Demo')
visTruck.SetWindowSize(1280, 1024)
visTruck.SetChaseCamera(trackPointTruck, 25.0, 1.5)
visTruck.Initialize()
visTruck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visTruck.AddLightDirectional()
visTruck.AddSkyBox()
visTruck.AttachVehicle(vehicleTruck.GetTractor())


driverTruck = veh.ChInteractiveDriverIRR(visTruck)


visSedan = veh.ChWheeledVehicleVisualSystemIrrlicht()
visSedan.SetWindowTitle('Sedan Demo')
visSedan.SetWindowSize(1280, 1024)
visSedan.SetChaseCamera(trackPointSedan, 25.0, 1.5)
visSedan.Initialize()
visSedan.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visSedan.AddLightDirectional()
visSedan.AddSkyBox()
visSedan.AttachVehicle(vehicleSedan.GetTractor())


driverSedan = veh.ChInteractiveDriverIRR(visSedan)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driverTruck.SetSteeringDelta(render_step_size / steering_time)
driverTruck.SetThrottleDelta(render_step_size / throttle_time)
driverTruck.SetBrakingDelta(render_step_size / braking_time)
driverSedan.SetSteeringDelta(render_step_size / steering_time)
driverSedan.SetThrottleDelta(render_step_size / throttle_time)
driverSedan.SetBrakingDelta(render_step_size / braking_time)


truck_states = []


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while visTruck.Run() and visSedan.Run():
    time = vehicleTruck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        visTruck.BeginScene()
        visTruck.Render()
        visTruck.EndScene()
        render_frame += 1

    
    driver_inputs = driverTruck.GetInputs()
    driver_inputs_sedan = driverSedan.GetInputs()

    
    driverTruck.Synchronize(time)
    terrain.Synchronize(time)
    vehicleTruck.Synchronize(time, driver_inputs)
    visTruck.Synchronize(time, driver_inputs)

    
    driverTruck.Advance(step_size)
    terrain.Advance(step_size)
    vehicleTruck.Advance(step_size)
    visTruck.Advance(step_size)

    
    truck_state = vehicleTruck.GetTractor().GetState()
    trailer_state = vehicleTruck.GetTrailer().GetState()
    truck_states.append((truck_state, trailer_state))

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

    
    driverSedan.Synchronize(time, driver_inputs_sedan)
    vehicleSedan.Synchronize(time, driver_inputs_sedan)
    driverSedan.Advance(step_size)
    visSedan.Advance(step_size)

    
    if step_number % 100 == 0:
        print(f"TRUCK STATE: {truck_states[-1][0]}")
        print(f"TRUCK TRAILER STATE: {truck_states[-1][1]}")