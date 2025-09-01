import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')







initLocKraz = chrono.ChVector3d(0, 3, 0.5)
initRotKraz = chrono.ChQuaterniond(0.866025, 0, 0, 0.5) 



vehicleKraz = veh.Kraz()
vehicleKraz.SetContactMethod(contact_method)
vehicleKraz.SetChassisCollisionType(chassis_collision_type)
vehicleKraz.SetChassisFixed(False)
vehicleKraz.SetInitPosition(chrono.ChCoordsysd(initLocKraz, initRotKraz))
vehicleKraz.Initialize()




tire_model = veh.TireModelType_RIGID  



initLocSedan = chrono.ChVector3d(10, 0, 0.5)
initRotSedan = chrono.ChQuaterniond(1, 0, 0, 0)

vehicleSedan = veh.Sedan(tire_model)
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



terrain = veh.RigidTerrain(vehicleKraz.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat,
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         veh.GetDataFile("terrain/meshes/highway.obj"))
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 60.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicleKraz.GetTractor())
vis.AttachVehicle(vehicleSedan.GetVehicle())






driverKraz = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driverKraz.SetSteeringDelta(render_step_size / steering_time)
driverKraz.SetThrottleDelta(render_step_size / throttle_time)
driverKraz.SetBrakingDelta(render_step_size / braking_time)

driverKraz.Initialize()


driverSedan = veh.ChDriver(vehicleSedan)
driverSedan.Initialize()






print("VEHICLE MASS: ", vehicleKraz.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


time_list = []
tractor_pos_x = []
tractor_pos_y = []
trailer_pos_x = []
trailer_pos_y = []

while vis.Run():
    time = vehicleKraz.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_kraz = driverKraz.GetInputs()

    
    driverSedan.SetSteering(0)
    driverSedan.SetThrottle(0.4)
    driverSedan.SetBraking(0)

    
    driverKraz.Synchronize(time)
    driverSedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicleKraz.Synchronize(time, driver_inputs_kraz, terrain)
    vehicleSedan.Synchronize(time, driverSedan.GetInputs(), terrain)
    vis.Synchronize(time, driver_inputs_kraz)

    
    driverKraz.Advance(step_size)
    driverSedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicleKraz.Advance(step_size)
    vehicleSedan.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

    
    time_list.append(time)
    tractor_pos_x.append(vehicleKraz.GetTractor().GetPos().x)
    tractor_pos_y.append(vehicleKraz.GetTractor().GetPos().y)

    if vehicleKraz.GetTrailer():
        trailer_pos_x.append(vehicleKraz.GetTrailer().GetPos().x)
        trailer_pos_y.append(vehicleKraz.GetTrailer().GetPos().y)


print("Time",time_list)
print("Tractor X Position", tractor_pos_x)
print("Tractor Y Position", tractor_pos_y)
print("Trailer X Position", trailer_pos_x)
print("Trailer Y Position", trailer_pos_y)