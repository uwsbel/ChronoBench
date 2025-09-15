import pychrono as chrono
import pychrono.irrlicht as chronoirr
from pychrono import ChCoordsysD, ChVectorD, Q_from_AngZ


system = chrono.ChSystemNSC()
system.Set_GravitationalAcceleration(ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(10, 0.1, 10)  
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)


ground_vis = chrono.ChBoxShape()
ground_vis.GetBoxGeometry().Size = ChVectorD(10, 0.1, 10)
ground.AddAsset(ground_vis)
system.Add(ground)


init_pos = ChVectorD(0, 0.2, 0)  
init_rot = Q_from_AngZ(0)        
turtlebot = chrono.ChTurtleBot(system)
turtlebot.Initialize(ChCoordsysD(init_pos, init_rot))


app = chronoirr.ChIrrApp(system, "Turtlebot Simulation", 800, 600)
app.AddTypicalCamera(chronoirr.vector3df(0, 2, -3))  
app.AddTypicalLights()
app.AddShadowAll()
app.AssetBindAll()
app.AssetUpdateAll()


driver = turtlebot.GetDriver()
time_step = 0.01


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    current_time = system.GetChTime()
    
    
    if current_time >= 2.0 and current_time < 4.0:
        driver.SetSpeeds(5, 3)  
    elif current_time >= 4.0:
        driver.SetSpeeds(5, 5)  
    
    
    turtlebot.Update()
    system.DoStepDynamics(time_step)