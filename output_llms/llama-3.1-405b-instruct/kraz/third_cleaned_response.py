import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


truck_initLoc = chrono.ChVector3d(0, 0, 0.5)
truck_initRot = chrono.ChQuaterniond(1, 0, 0, 0)

sedan_initLoc = chrono.ChVector3d(10, 0, 0.5)
sedan_initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


truck_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY


terrain_model = veh.RigidTerrain.MESH
terrain_mesh = veh.GetDataFile("terrain/meshes/highway.obj")
terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0,0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_initLoc, truck_initRot))
truck.SetTireModel(truck_tire_model)
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_initLoc, sedan_initRot))
sedan.SetTireModel(sedan_tire_model)
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrain_mesh)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan.GetChassis())


truck_driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
truck_driver.SetSteeringDelta(render_step_size / steering_time)
truck_driver.SetThrottleDelta(render_step_size / throttle_time)
truck_driver.SetBrakingDelta(render_step_size / braking_time)

truck_driver.Initialize()


sedan_driver = veh.ChDriver()
sedan_driver.SetThrottle(0.5)
sedan_driver.SetSteering(0.0)
sedan_driver.Initialize()


print( "TRUCK MASS: ",  truck.GetTractor().GetMass())
print( "SEDAN MASS: ",  sedan.GetChassis().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

truck_states = []
sedan_states = []

while vis.Run() :
    time = truck.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    truck_driver_inputs = truck_driver.GetInputs()

    
    truck_driver.Synchronize(time)
    sedan_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_driver_inputs, terrain)
    sedan.Synchronize(time, sedan_driver.GetInputs(), terrain)
    vis.Synchronize(time, truck_driver_inputs)

    
    truck_driver.Advance(step_size)
    sedan_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    truck_states.append((truck.GetTractor().GetPos(), truck.GetTractor().GetRot()))
    sedan_states.append((sedan.GetChassis().GetPos(), sedan.GetChassis().GetRot()))

    
    step_number += 1

    
    realtime_timer.Spin(step_size)


with open("truck_states.txt", "w") as f:
    for state in truck_states:
        f.write(str(state[0].x) + " " + str(state[0].y) + " " + str(state[0].z) + " " + str(state[1].e0) + " " + str(state[1].e1) + " " + str(state[1].e2) + " " + str(state[1].e3) + "\n")

with open("sedan_states.txt", "w") as f:
    for state in sedan_states:
        f.write(str(state[0].x) + " " + str(state[0].y) + " " + str(state[0].z) + " " + str(state[1].e0) + " " + str(state[1].e1) + " " + str(state[1].e2) + " " + str(state[1].e3) + "\n")