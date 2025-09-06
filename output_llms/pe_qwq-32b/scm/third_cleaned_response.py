import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random  


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(-8, 0, 0.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVectorD(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
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


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
for _ in range(10):
    while True:
        x = random.uniform(-40, 40)
        y = random.uniform(-40, 40)
        distance = math.sqrt((x + 8)**2 + y**2)
        if distance > 5:  
            break
    z = terrainHeight + 0.5  
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVectorD(x, y, z))
    vehicle.GetSystem().Add(box)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,   
                          0,     
                          1.1,   
                          0,     
                          30,    
                          0.01,  
                          2e8,   
                          3e4)   
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)  


vis = irr.ChIrrApp(vehicle.GetSystem(), 'HMMWV Demo', irr.dimension2du(1280, 1024))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.SetCameraPosition(chrono.ChVectorD(0, 3, 15))  
vis.SetChaseCamera(trackPoint, 6.0, 0.5)


vis.AddLightPoint(chrono.ChVectorD(20, 20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(-20, -20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(0, 30, 10), chrono.ChColor(1, 1, 1), 2.0)


camera = vis.GetCamera()
camera.SetFov(60)  

vis.AddSkyBox()
vis.EnableShadowVolumes()
vis.SetSymbolsScale(0.01)
vis.SetCameraSensitivity(0.001, 0.001)
vis.SetTimestep(step_size)
vis.Initialize()


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()
print("VEHICLE MASS: ", vehicle.GetMass())  

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    
    
    vis Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random


veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(-8, 0, 0.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID
contact_method = chrono.ChContactMethod_SMC
step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
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


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
for _ in range(10):
    while True:
        x = random.uniform(-40, 40)
        y = random.uniform(-40, 40)
        distance = math.sqrt((x + 8)**2 + y**2)
        if distance > 5:
            break
    z = 0.5
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, material)
    box.SetPos(chrono.ChVectorD(x, y, z))
    vehicle.GetSystem().Add(box)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)


vis = irr.ChIrrApp(vehicle.GetSystem(), 'Enhanced HMMWV Simulation', irr.dimension2du(1280, 1024))
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.SetCameraPosition(chrono.ChVectorD(0, 3, 15))
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 1.71), 6.0, 0.5)


vis.AddLightPoint(chrono.ChVectorD(20, 20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(-20, -20, 10), chrono.ChColor(1, 1, 1), 2.0)
vis.AddLightPoint(chrono.ChVectorD(0, 30, 10), chrono.ChColor(1, 1, 1), 2.0)


camera = vis.GetCamera()
camera.SetFov(60)
vis.AddSkyBox()
vis.EnableShadowVolumes()
vis.SetSymbolsScale(0.01)
vis.SetCameraSensitivity(0.001, 0.001)
vis.SetTimestep(step_size)
vis.Initialize()


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()
print("VEHICLE MASS:", vehicle.GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    driver_inputs = driver.GetInputs()
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)