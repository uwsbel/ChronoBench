import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType.MESH
chassis_collision_type = veh.CollisionType.NONE
tire_model = veh.TireModelType.TMEASY


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50


vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(10, 0, 0.5), initRot))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()
vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)


driver1 = veh.ChDriver()
driver2 = veh.ChDriver()
vehicle.SetDriver(driver1)
vehicle2.SetDriver(driver2)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVectorD(-5.0, 0.0, 1.8), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)
vis.AttachVehicle(vehicle2)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    steering_amplitude = 0.5
    steering_freq = 0.5  
    steering1 = steering_amplitude * math.sin(2 * math.pi * steering_freq * time)
    steering2 = steering_amplitude * math.sin(2 * math.pi * steering_freq * time + math.pi/2)
    driver1.SetSteering(steering1)
    driver2.SetSteering(steering2)

    
    driver1_inputs = driver1.GetInputs()
    driver2_inputs = driver2.GetInputs()

    
    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver1_inputs, terrain)
    vehicle2.Synchronize(time, driver2_inputs, terrain)
    vis.Synchronize(time)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vehicle2.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)