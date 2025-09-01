import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')





truck_init_loc = chrono.ChVector3d(-10.0,  0.0, 0.5)   
truck_init_rot = chrono.Q_from_AngZ(math.radians(5.0))  


sedan_init_loc = chrono.ChVector3d( 20.0, -3.5, 0.45)   
sedan_init_rot = chrono.QUNIT                               




step_size                 = 1e-3
render_step_size          = 1.0/50.0      
tire_step_size            = step_size

contact_method            = chrono.ChContactMethod_NSC
chassis_collision_type    = veh.CollisionType_NONE
vis_type                  = veh.VisualizationType_MESH       




truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truck_init_loc, truck_init_rot))


truck.SetTireType(veh.TireModelType_RIGID)
truck.SetTireStepSize(tire_step_size)

truck.Initialize()


truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)


truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedan_init_loc, sedan_init_rot))


sedan.SetTireType(veh.TireModelType_TMEASY)
sedan.SetTireStepSize(tire_step_size)

sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)




sys = truck.GetSystem()             
terrain = veh.RigidTerrain(sys)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


mesh_file = veh.GetDataFile("terrain/meshes/highway.obj")
patch = terrain.AddPatch(patch_mat,
                         mesh_file,
                         chrono.ChVector3d(0, 0, 0),          
                         chrono.QUNIT)                        
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
patch.SetTexture(veh.GetDataFile("terrain/textures/road.jpg"), 1, 1)

terrain.Initialize()




track_point = chrono.ChVector3d(0, 0, 2.2)   
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz + Sedan on Highway")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(track_point, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddLightDirectional()


vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(truck.GetTrailer())
vis.AttachVehicle(sedan.GetVehicle())




truck_driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0     
throttle_time = 1.0
braking_time  = 0.3

truck_driver.SetSteeringDelta (render_step_size / steering_time)
truck_driver.SetThrottleDelta (render_step_size / throttle_time)
truck_driver.SetBrakingDelta  (render_step_size / braking_time)
truck_driver.Initialize()




SEDAN_THROTTLE = 0.5
SEDAN_STEER    = 0.0
SEDAN_BRAKE    = 0.0
sedan_driver_inputs = veh.DriverInputs()
sedan_driver_inputs.m_throttle = SEDAN_THROTTLE
sedan_driver_inputs.m_steering = SEDAN_STEER
sedan_driver_inputs.m_braking  = SEDAN_BRAKE




tractor_states = []     
trailer_states = []




render_steps   = math.ceil(render_step_size / step_size)
step_number    = 0
realtime_timer = chrono.ChRealtimeStepTimer()

print("Truck tractor mass: ",  truck.GetTractor().GetMass())
print("Truck trailer mass: ",  truck.GetTrailer().GetMass())
print("Sedan vehicle mass:",   sedan.GetVehicle().GetMass())

while vis.Run():
    time = sys.GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        tractor_states.append((time,
                               truck.GetTractor().GetPos(),
                               truck.GetTractor().GetRot()))
        trailer_states.append((time,
                               truck.GetTrailer().GetPos(),
                               truck.GetTrailer().GetRot()))

    
    
    
    truck_inputs = truck_driver.GetInputs()          
    sedan_inputs = sedan_driver_inputs               

    
    
    
    truck_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, truck_inputs)              

    
    
    
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    
    
    step_number += 1
    realtime_timer.Spin(step_size)




with open("truck_states.txt", "w") as f:
    for (t, pos, rot) in tractor_states:
        f.write(f"{t:.6f} {pos.x:.6f} {pos.y:.6f} {pos.z:.6f} "
                f"{rot.e0:.6f} {rot.e1:.6f} {rot.e2:.6f} {rot.e3:.6f}\n")

with open("trailer_states.txt", "w") as f:
    for (t, pos, rot) in trailer_states:
        f.write(f"{t:.6f} {pos.x:.6f} {pos.y:.6f} {pos.z:.6f} "
                f"{rot.e0:.6f} {rot.e1:.6f} {rot.e2:.6f} {rot.e3:.6f}\n")