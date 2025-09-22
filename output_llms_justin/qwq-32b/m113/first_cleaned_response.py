import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


material = chrono.ChMaterialSurface()
material.SetFriction(0.9)
material.SetRestitution(0.1)

terrain = veh.ChOffroadRigidTerrain(system)
terrain.SetMaterialSurface(material)
terrain.Initialize()


vehicle = veh.M113()
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
vehicle.Initialize(terrain, chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0))


driver = veh.ChDriver()
driver.SetThrottle(0.6)  
driver.SetSteering(0)    
vehicle.SetDriver(driver)


app = chronoirr.ChIrrApp(system, 'M113 Simulation', chronoirr.dimension2du(1024, 768))
app.AddLightWithShadow(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0), 2, 60, 200, 100)
app.AddSkyBox()
app.AddTypicalLights()
app.SetCameraPosition(chrono.ChVectorD(5, -10, 2))
app.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Y))
app.AssetBind()
app.AssetUpdate()
app.Add(terrain.GetCollisionModel())
app.Add(vehicle.GetVisualization())


time_step = 0.01
real_time_factor = 1  


while app.GetDevice().run():
    time = system.GetChTime()
    
    
    driver.Update(time_step)
    
    
    vehicle.Update(time)
    system.DoStepDynamics(time_step)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    app.SetTimestep(time_step)
    app.DoStep()
    chronoirr.core.SleepChrono(time_step)