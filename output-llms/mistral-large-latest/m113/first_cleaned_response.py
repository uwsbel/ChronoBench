import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()


terrain = veh.RigidTerrain(system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
system.Add(terrain)


m113 = veh.M113()
m113.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT))
m113.SetInitWheelAngVel(-10)
m113.SetInitFwdVel(1)


driver = veh.ChDriver(m113.GetVehicle())
driver.Initialize()


app = chronoirr.ChIrrApp(m113.GetVehicle().GetSystem(), "M113 Vehicle Simulation", chrono.ChVectorD(2.5, 2.5, 2.5), 0.5)
app.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
app.AddTypicalLights()
app.AddTypicalCamera(chrono.ChVectorD(0, 2, -6))
app.SetShowInfos(True)
app.SetTimestep(0.01)
app.SetTryRealtime(True)


realtime_timer = chrono.ChRealtimeStepTimer()
app.SetTimer(realtime_timer)

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    
    driver.Synchronize(m113.GetVehicle())

    
    app.DoStep()

    
    app.EndScene()