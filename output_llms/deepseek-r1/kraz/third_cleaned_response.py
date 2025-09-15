import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc_truck = chrono.ChVector3d(-5, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(5, 2, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model_truck = veh.TireModelType_RIGID
tire_model_sedan = veh.TireModelType_TMEASY


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
render_step_size = 1.0 / 50


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.Initialize()


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.Initialize()


terrain = veh.RigidTerrain(truck.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.CSYSNORM, veh.GetDataFile("terrain/meshes/highway.obj"), "highway", 0.01)
terrain.Initialize()


truck.InitializeTires(tire_model_truck, veh.TireStepType_NORMAL, terrain)
sedan.InitializeTires(tire_model_sedan, veh.TireStepType_NORMAL, terrain)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.1), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AddVehicle(sedan.GetVehicle())


driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_sedan = veh.ChDriver(sedan.GetVehicle())


driver_truck.SetSteeringDelta(render_step_size / 1.0)
driver_truck.SetThrottleDelta(render_step_size / 1.0)
driver_truck.SetBrakingDelta(render_step_size / 0.3)
driver_truck.Initialize()


driver_sedan.SetThrottle(0.5)
driver_sedan.SetSteering(0.0)
driver_sedan.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = math.ceil(render_step_size / step_size)
tractor_states = []
trailer_states = []

while vis.Run():
    time = truck.GetSystem().GetChTime()
    
    
    if vis.ProcessEvents():
        vis.Synchronize(time, driver_truck.GetInputs())
        vis.Render()

    
    tractor_states.append(truck.GetTractor().GetPos())
    if truck.GetTrailer():
        trailer_states.append(truck.GetTrailer().GetPos())

    
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    truck.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)