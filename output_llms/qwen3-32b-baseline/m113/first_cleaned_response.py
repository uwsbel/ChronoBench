import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irc as chronoirr
import time


sys = chrono.ChSystemNSC()
sys.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))


m113 = vehicle.WheeledVehicle(sys, vehicle.GetM113())
m113.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.5), chrono.Q_from_Euler_angles(0, 0, 0)))
m113.Initialize()


terrain = vehicle.RigidTerrain(sys)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(100, 100, 1))
patch.SetFriction(0.9)
patch.SetRestitution(0.1)
terrain.Initialize()


app = chronoirr.ChIrrApp(sys, "M113 Simulation", chronoirr.dimension2d(800, 600))
app.SetSkyBox()
app.SetCamera(chronoirr.vector3df(0, 2, -6), chronoirr.vector3df(0, 0, 1))
app.SetAmbientLight(chronoirr.SColorf(1, 1, 1, 1))
app.AddTypicalLights()
app.SetShadowMode(chronoirr.EShadowMode.EShadow_All)

driver = vehicle.ChIrrGuiDriver(app)
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)


time_step = 0.001
sim_time = 0
real_time = 0.0

while app.GetDevice().run():
    start_time = time.time()
    
    app.BeginScene()
    app.DrawAll()
    
    
    driver.Update()
    
    
    m113.Synchronize(sim_time, driver.GetInputs(), terrain)
    
    
    sys.DoStepDynamics(time_step)
    sim_time += time_step
    
    app.EndScene()
    
    
    step_time = time.time() - start_time
    if step_time < time_step:
        time.sleep(time_step - step_time)
    real_time = time.time() - start_time