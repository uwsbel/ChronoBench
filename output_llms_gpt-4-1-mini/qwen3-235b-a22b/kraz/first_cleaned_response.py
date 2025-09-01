import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



vehicle_file = veh.GetDataFile("vehicle/Kraz/Kraz_Vehicle.json")
powertrain_file = veh.GetDataFile("vehicle/Kraz/Kraz_Powertrain.json")
tire_file = veh.GetDataFile("vehicle/Kraz/Kraz_Tire.json")

vehicle = veh.WheeledVehicle(system, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
vehicle.SetPowertrain(veh.SimplePowertrain(powertrain_file))
vehicle.SetTireModels([veh.RigidTire(tire_file) for _ in range(vehicle.GetNumberAxles())])
vehicle.GetChassis().SetMass(5000)  


terrain_mat = chrono.ChMaterialSurfaceNSC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(terrain_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(200, 200, 1))
terrain.Initialize()


driver = veh.ChDriver(vehicle)
driver.Initialize()


app = veh.ChWheeledVehicleIrrApp(vehicle, 'Kraz Vehicle Simulation', irr.dimension2du(1280, 1024))
app.SetSkyBox()
app.AddTypicalLights()
app.SetChaseCamera(chrono.ChVectorD(0, 2, -5), 6.0, 0.5)
app.GuiScriptInit()


step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()

while app.GetDevice().run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    app.Synchronize('Kraz Simulation', driver_inputs)

    
    step = realtime_timer.SuggestTimeStep(step_size)
    driver.Advance(step)
    vehicle.Advance(step)
    terrain.Advance(step)
    app.Advance(step)

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()