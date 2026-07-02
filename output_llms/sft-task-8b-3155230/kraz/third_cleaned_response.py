import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, -10, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrain_length = 100.0
terrain_width = 100.0


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle_sedan = veh.Sedan()
vehicle_sedan.SetContactMethod(contact_method)
vehicle_sedan.SetChassisCollisionType(chassis_collision_type)
vehicle_sedan.SetChassisFixed(False)
vehicle_sedan.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 10, 0.5), initRot))
vehicle_sedan.Initialize()

vehicle_sedan.SetChassisVisualizationType(vis_type, vis_type)
vehicle_sedan.SetSteeringVisualizationType(vis_type)
vehicle_sedan.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_sedan.SetWheelVisualizationType(vis_type, vis_type)
vehicle_sedan.SetTireVisualizationType(vis_type, vis_type)


terrain = veh.MeshTerrain(vehicle.GetSystem())
terrain.LoadMesh(chrono.GetChronoDataFile('vehicle/terrain/highway.obj'))
terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())


driver = veh.ChInteractiveDriverIRR(vis)
driver_sedan = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


driver_sedan.SetSteeringDelta(0.0)
driver_sedan.SetThrottleDelta(0.0)
driver_sedan.SetBrakingDelta(0.0)
driver_sedan.Initialize()


print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


tractor_positions = []
tractor_orientations = []
trailer_positions = []
trailer_orientations = []

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    
    driver.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vehicle_sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vehicle_sedan.Advance(step_size)
    vis.Advance(step_size)

    
    tractor_pos = vehicle.GetTractor().GetPos()
    tractor_rot = vehicle.GetTractor().GetRot()
    trailer_pos = vehicle.GetTrailer().GetPos()
    trailer_rot = vehicle.GetTrailer().GetRot()
    tractor_positions.append(tractor_pos)
    tractor_orientations.append(tractor_rot)
    trailer_positions.append(trailer_pos)
    trailer_orientations.append(trailer_rot)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)