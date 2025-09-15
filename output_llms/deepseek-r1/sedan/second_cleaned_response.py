import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50  
contact_method = chrono.ChContactMethod_NSC


system = chrono.ChSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle1 = veh.BMW_E90(system)
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle1.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle1.SetTireType(veh.TireModelType_TMEASY)
vehicle1.SetTireStepSize(tire_step_size)
vehicle1.Initialize()
vehicle1.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle1.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle1.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle1.SetTireVisualizationType(veh.VisualizationType_MESH)


vehicle2 = veh.BMW_E90(system)
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle2.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(5, 0, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle2.SetTireType(veh.TireModelType_TMEASY)
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()
vehicle2.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle2.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle2.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle2.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, 
                       chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                       100.0, 100.0)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Vehicles Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(-5.0, 0.0, 1.8), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle1.GetVehicle())
vis.AttachVehicle(vehicle2.GetVehicle())


driver1 = veh.ChExternalDriver()
driver2 = veh.ChExternalDriver()
driver1.Initialize()
driver2.Initialize()


realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = math.ceil(render_step_size / step_size)
steering_amplitude = 0.75
steering_frequency = 0.5

while vis.Run():
    time = system.GetChTime()
    
    
    steering = steering_amplitude * math.sin(2 * math.pi * steering_frequency * time)
    driver1.SetSteering(steering)
    driver2.SetSteering(steering)
    
    
    if vis.ProcessEvents():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    
    driver_inputs1 = driver1.GetInputs()
    driver_inputs2 = driver2.GetInputs()
    
    driver1.Synchronize(time)
    driver2.Synchronize(time)
    terrain.Synchronize(time)
    vehicle1.Synchronize(time, driver_inputs1, terrain)
    vehicle2.Synchronize(time, driver_inputs2, terrain)
    vis.Synchronize(time, driver_inputs1)
    
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    vis.Advance(step_size)
    
    system.DoStepDynamics(step_size)
    realtime_timer.Spin(step_size)