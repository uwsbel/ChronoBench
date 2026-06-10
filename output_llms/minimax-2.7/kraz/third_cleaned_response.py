import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")




contact_method = chrono.ChContactMethod_NSC   
contact_vis    = False

step_size      = 1e-3               
tire_step_size = step_size

render_step_size = 1.0 / 50         
render_steps    = math.ceil(render_step_size / step_size)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE




initLoc = chrono.ChVector3d(5.0, -2.0, 0.5)                
initRot = chrono.ChQuaterniond(0.9659, 0.0, 0.0, 0.2588)  

trackPoint = chrono.ChVector3d(0.0, 0.0, 2.1)   


tire_model = veh.TireModelType_RIGID




vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetTireModel(tire_model)                         
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




terrain = veh.RigidTerrain(vehicle.GetSystem())


highway_mesh = chrono.ChTriangleMeshConnected()
chrono.ImportMesh(
    chrono.GetChronoDataFile("terrain/meshes/hw_road.obj"), highway_mesh
)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patch = terrain.AddMesh(highway_mesh, patch_mat, 0, True)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz + Sedan Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())




driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   
throttle_time = 1.0  
braking_time   = 0.3  
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

print("VEHICLE MASS (truck):", vehicle.GetTractor().GetMass())




sedan_initLoc = chrono.ChVector3d(-10.0, 0.0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)   

sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)




sedan_driver = veh.ChDriver(sedan)
sedan_driver.Initialize()




tractor_positions = []
tractor_rotations = []
trailer_positions = []
trailer_rotations = []
simulation_times  = []




realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

        
        simulation_times.append(time)
        tractor = vehicle.GetTractor()
        trailer = vehicle.GetTrailer()
        tractor_positions.append(tractor.GetPos())
        tractor_rotations.append(tractor.GetRot())
        trailer_positions.append(trailer.GetPos())
        trailer_rotations.append(trailer.GetRot())

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs)          
    vis.Synchronize(time, driver_inputs)

    
    sedan_driver.SetSteering(0.0)      
    sedan_driver.SetThrottle(0.3)      
    sedan_driver.SetBraking(0.0)
    sedan_driver.Synchronize(time)
    sedan.Synchronize(time, sedan_driver.GetInputs())

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    sedan_driver.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)




print("\nSimulation finished.")
print(f"Collected {len(simulation_times)} frames.")
if simulation_times:
    print("First tractor position:", tractor_positions[0])
    print("Last  trailer position:", trailer_positions[-1])