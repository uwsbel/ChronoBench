import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, -5, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


sedanLoc = chrono.ChVectorD(0, 5, 0.5)
sedanRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50


vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)
vehicle.SetTireType(veh.TireModelType_RIGID)  


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
sedan.SetInitPosition(chrono.ChCoordsysD(sedanLoc, sedanRot))
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddMeshPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight)),
    veh.GetDataFile("terrain/highway.obj"), 1.0, 1.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Truck and Sedan Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())
vis.AttachVehicle(sedan)  


truck_driver = veh.ChInteractiveDriverIRR(vis)
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()


sedan_driver = veh.ChDriver()
sedan_driver.SetThrottle(0.5)  
sedan_driver.SetSteering(0.0)  
sedan.SetDriver(sedan_driver)


tractor_states = []
trailer_states = []


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    truck_inputs = truck_driver.GetInputs()
    vehicle.Synchronize(time, truck_inputs, terrain)
    
    
    
    
    truck_driver.Advance(step_size)
    sedan.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)
    
    
    tractor = vehicle.GetTractor()
    trailer = vehicle.GetTrailer()
    tractor_states.append((time, tractor.GetPos(), tractor.GetRot()))
    trailer_states.append((time, trailer.GetPos(), trailer.GetRot()))

    step_number += 1
    realtime_timer.Spin(step_size)