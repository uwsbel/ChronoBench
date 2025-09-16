import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.M113()
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT), system)


friction = 0.9
restitution = 0.1
material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(friction)
material.SetRestitution(restitution)
terrain = veh.RigidTerrain(system, material)
terrain.Initialize()


driver = veh.Driver()
vehicle.SetDriver(driver)


app = irr.ChIrrApp(system, 'M113 Simulation', irr.dimension2du(1280, 1024))
app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
app.AddTypicalSky()
app.AddTypicalLights()
app.SetCamera(chrono.ChVectorD(5, 0, 2), chrono.ChVectorD(0, 0, 0))
app.SetSymbolscale(0.002)
app.SetTimestep(0.01)  


app.AddAsset(vehicle.GetVisualization())
terrain.Visualize(app)


app.Initialize()


step_time = 0.01  

while app.GetDevice().run():
    driver.Update(step_time)
    system.DoStepDynamics(step_time)
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    app.DoStep()