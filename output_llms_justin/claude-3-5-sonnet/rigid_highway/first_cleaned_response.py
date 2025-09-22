import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.RigidTerrain(system)
terrain_file_vis = "Highway_vis.obj"
terrain_file_col = "Highway_col.obj"

terrain_mesh = terrain.AddMesh(chrono.GetChronoDataFile(terrain_file_col),
                               chrono.GetChronoDataFile(terrain_file_vis),
                               chrono.ChVectorD(0, 0, 0))
terrain_mesh.SetContactFrictionCoefficient(0.8)
terrain.Initialize()


vehicle_file = veh.GetDataPath() + "hmmwv/vehicle/HMMWV_Vehicle.json"
tire_file = veh.GetDataPath() + "hmmwv/tire/HMMWV_TMeasyTire.json"
powertrain_file = veh.GetDataPath() + "hmmwv/powertrain/HMMWV_ShaftsPowertrain.json"

vehicle = veh.WheeledVehicle(system, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


for axle in vehicle.GetAxles():
    tireL = veh.TMeasyTire(tire_file)
    vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
    tireR = veh.TMeasyTire(tire_file)
    vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)


powertrain = veh.ReadPowertrainJSON(powertrain_file)
vehicle.InitializePowertrain(powertrain)


driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


vis = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), "HMMWV Simulation")
vis.SetHUDLocation(500, 20)
vis.SetSkyBox()
vis.AddTypicalLogo()
vis.AddTypicalLights(irr.vector3df(30, -30, 100), irr.vector3df(30, 50, 100), 250, 130)
vis.AddTypicalCamera(irr.vector3df(6, 3, -6))
vis.AttachVehicle(vehicle.GetVehicle())


step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()


while vis.GetDevice().run():
    time = vehicle.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    
    if time > 1000:  
        break


vis.GetDevice().drop()