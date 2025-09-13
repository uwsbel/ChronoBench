import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 0.1, 20, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


init_pos = chrono.ChVectorD(0, 0.1, 0)
init_rot = chrono.Q_from_AngZ(math.pi / 4)  
turtlebot = robot.TurtleBot()
turtlebot.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
turtlebot.Initialize(system)


application = chronoirr.ChIrrApp(system, "TurtleBot Simulation", chronoirr.dimension2du(1280, 720))
application.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddSkyBox()
application.AddTypicalLights()


camera_pos = chrono.ChVectorD(3, 2, 3)
camera_target = init_pos
application.AddTypicalCamera(chronoirr.vector3df(camera_pos.x, camera_pos.y, camera_pos.z),
                             chronoirr.vector3df(camera_target.x, camera_target.y, camera_target.z))

application.AssetBindAll()
application.AssetUpdateAll()


application.SetTimestep(0.01)
time = 0

while application.GetDevice().run():
    time = system.GetChTime()
    
    
    if time < 2.0:
        
        turtlebot.SetMotorSpeeds(1.0, 1.0)
    elif time < 4.0:
        
        turtlebot.SetMotorSpeeds(0.5, 1.5)
    elif time < 6.0:
        
        turtlebot.SetMotorSpeeds(1.5, 0.5)
    else:
        
        turtlebot.SetMotorSpeeds(0, 0)
    
    
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    application.DoStep()
    application.EndScene()