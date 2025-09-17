import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(20, 0.5, 20)  
ground.SetCollide(True)


box = chrono.ChBoxShape()
box.GetBoxGeometry().Size = chrono.ChVectorD(20, 0.5, 20)
ground.AddAsset(box)
texture = chrono.ChTexture()
texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(texture)


system.Add(ground)


turtlebot = robot.ChTurtlebot(system, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0), 
                                                    chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 1, 0))))


application = irr.ChIrrApp(system, 'Turtlebot Simulation', irr.dimension2du(800, 600))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), chrono.ChVectorD(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


while application.GetDevice().run():
    time = system.GetChTime()

    
    if time < 1.0:
        
        turtlebot.SetMotorSpeed(0, 10)
        turtlebot.SetMotorSpeed(1, 10)
    elif time < 2.0:
        
        turtlebot.SetMotorSpeed(0, 5)
        turtlebot.SetMotorSpeed(1, 10)
    else:
        
        turtlebot.SetMotorSpeed(0, 10)
        turtlebot.SetMotorSpeed(1, 5)
    
    
    system.DoStepDynamics(1e-3)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()