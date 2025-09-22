import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import os






chrono_data_dir = chrono.GetChronoDataFile("../data/")


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)






man_file = chrono.GetChronoDataFile("vehicle/man/MAN_10t.json")
vehicle = veh.WheeledVehicle(system, man_file)


vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.7, 0), chrono.ChQuaternionD(1, 0, 0, 0)))


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


vehicle.SetChassisCollide(False)






tire_file = chrono.GetChronoDataFile("vehicle/tmeasy/truck_TMeasy.json")
tires = []

for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        tire = veh.TMeasyTire(tire_file)
        tire.SetVisualizationType(veh.VisualizationType_MESH)
        tire.Initialize(wheel)
        tires.append(tire)
        system.Add(tire)






terrain = veh.RigidTerrain(system)


terrain_file = chrono.GetChronoDataFile("vehicle/terrain/RigidPlane.json")
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


patch = terrain.AddPatch(patch_mat, 
                        chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), 
                        200, 200)


patch.SetTexture(chrono.GetChronoDataFile("textures/dirt.jpg"), 200, 200)
terrain.Initialize()






driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(1.0 / 50)
driver.SetThrottleDelta(1.0 / 50)
driver.SetBrakingDelta(1.0 / 50)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.5   
driver.SetSteeringDelta(steering_time / 50)
driver.SetThrottleDelta(throttle_time / 50)
driver.SetBrakingDelta(braking_time / 50)

driver.Initialize()






vis_width = 1280
vis_height = 720
vis = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), "MAN 10t Truck Simulation", 
                                 chrono.ChVectorD(0, 0, 0), 
                                 vis_width, vis_height)

vis.SetChaseCamera(6.0, 0.5, 0.5)
vis.SetChaseCameraState(veh.ChChaseCamera.Track)
vis.SetChaseCameraAngle(0.0)


vis.GetCamera().setPosition(irr.vector3df(3, 5, -8))
vis.GetCamera().setTarget(irr.vector3df(0, 0, 0))


vis.AddTypicalLights()
vis.GetDevice().getSceneManager().setAmbientLight(irr.SColorf(0.1, 0.1, 0.1, 1.0))
vis.AddSkyBox()


vis.AddLogo(chrono.GetChronoDataFile("logo/chronologo.png"))


vis.SetDriverMode(driver.GetInputMode())


vis.Initialize()
vis.AddVehicleToScene(vehicle.GetVehicle())






step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()





while vis.GetDevice().run():
    
    driver_inputs = driver.GetInputs()
    
    
    time = system.GetChTime()
    vis.BeginScene()
    vis.Render()
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    
    for tire in tires:
        tire.Synchronize(time, terrain)
    
    
    terrain.Synchronize(time)
    
    
    vis.Synchronize(time, driver_inputs)
    
    
    vehicle.Advance(step_size)
    for tire in tires:
        tire.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    vis.Advance(step_size)
    
    
    vis.EndScene()
    
    
    realtime_timer.Spin(step_size)





vis.GetDevice().drop()