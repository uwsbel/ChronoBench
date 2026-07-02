import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr


sys = chrono.ChSystemNSC()
sys.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


kraz = vehicle.Kraz(sys)
kraz.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.5), chrono.Q_from_AngZ(0)))
kraz.SetAutomaticTrans(True)
kraz.Initialize()


terrain = vehicle.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)), 200, 200)
terrain.GetGroundBody().GetMaterialSurfaceNSC().SetFrictionCoefficient(0.9)
terrain.GetGroundBody().GetMaterialSurfaceNSC().SetRestitutionCoefficient(0.1)
terrain.Initialize()


driver = vehicle.ChDriver(kraz)
driver.SetThrottle(0.5)
driver.SetSteering(0)
driver.SetBraking(0)


app = irr.ChIrrApp(sys, "Kraz Simulation", irr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(chrono.ChVectorD(0, 3, -10), chrono.ChVectorD(0, 0, 1.5))
app.AddTypicalLights()
app.SetShowInfos(True)


time_step = 0.02
simulation_time = 0


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(simulation_time)
    kraz.Synchronize(simulation_time, driver_inputs)
    terrain.Synchronize(simulation_time)

    
    sys.DoStepDynamics(time_step)
    simulation_time += time_step

    app.EndScene()
    irr.ChRealtimeStep(time_step)