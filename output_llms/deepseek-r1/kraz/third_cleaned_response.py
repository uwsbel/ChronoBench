import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystem()
contact_method = chrono.ChContactMethod_NSC
system.SetContactMethod(contact_method)


initLoc_truck = chrono.ChVector3d(0, 2, 0.5)  
initLoc_sedan = chrono.ChVector3d(0, -2, 0.5)  
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID  


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(0, 0, 2.1)


step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot))
truck.Initialize(system)
truck.SetChassisVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)
truck.SetTireType(tire_model)  


sedan = veh.Sedan()
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot))
sedan.Initialize(system)
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)
sedan.SetTireType(tire_model)  


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/terrain/meshes/highway.obj"))  
terrain.AddMesh(patch_mat, mesh)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())


driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_sedan = veh.ChDriver(sedan.GetVehicle())


steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()


driver_sedan.SetThrottle(0.5)  
driver_sedan.SetSteering(0.0)  
driver_sedan.SetBraking(0.0)


tractor_file = open("truck_tractor.out", "w")
trailer_file = open("truck_trailer.out", "w")


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = system.GetChTime()
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    inputs_truck = driver_truck.GetInputs()
    inputs_sedan = driver_sedan.GetInputs()

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, inputs_truck, terrain)
    sedan.Synchronize(time, inputs_sedan, terrain)
    vis.Synchronize(time, inputs_truck)

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    tractor_body = truck.GetTractor().GetChassisBody()
    trailer_body = truck.GetTrailer().GetChassisBody()
    
    tractor_pos = tractor_body.GetPos()
    tractor_rot = tractor_body.GetRot()
    trailer_pos = trailer_body.GetPos()
    trailer_rot = trailer_body.GetRot()
    
    tractor_file.write(f"{time} {tractor_pos.x} {tractor_pos.y} {tractor_pos.z} "
                      f"{tractor_rot.e0} {tractor_rot.e1} {tractor_rot.e2} {tractor_rot.e3}\n")
    trailer_file.write(f"{time} {trailer_pos.x} {trailer_pos.y} {trailer_pos.z} "
                      f"{trailer_rot.e0} {trailer_rot.e1} {trailer_rot.e2} {trailer_rot.e3}\n")

    step_number += 1
    realtime_timer.Spin(step_size)


tractor_file.close()
trailer_file.close()