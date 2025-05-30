import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.FEDA(system)
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.SetTireType(veh.TIRE_MODEL_TMEASY)
vehicle.Initialize()


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         veh.RigidTerrain.PATCH_BOX, 200, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"))
terrain.Initialize()


application = chronoirr.ChIrrApp(system, 'FEDA Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddSkyBox()
application.AddCamera(chronoirr.vector3df(0, 3, -6), chronoirr.vector3df(0, 0, 0))
chase_object = vehicle.GetChassisBody()
application.SetChaseCamera(chase_object, 6.0, 0.5)


application.AssetBindAll()
application.AssetUpdateAll()


driver = veh.ChIrrGuiDriver(application)
driver.Initialize()


system.SetStep(1.0 / 50.0)


while application.GetDevice().run():
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs)
    system.DoStepDynamics(system.GetStep())
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()