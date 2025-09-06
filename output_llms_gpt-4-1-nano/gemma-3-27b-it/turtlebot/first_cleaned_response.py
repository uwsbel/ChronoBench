import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.vehicle as veh
import math


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  


body_ground = chrono.ChBody()
body_ground.SetBodyFixed(True)
body_ground.SetPos(chrono.ChVector3d(0, -0.1, 0))
body_ground.SetShapeFromBox(10, 0.1, 10)
body_ground.CreateMaterialAndAddToBody(chrono.ChMaterialSurfaceNSC())
sys.Add(body_ground)


terrain = veh.RigidTerrain(sys)
terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 20, 20)
terrain.Initialize()


turtlebot = robot.Turtlebot(sys)
turtlebot.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.1, 0), chrono.QUNIT))
turtlebot.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 2, -5))
vis.AddTypicalLights()


time_step = 0.01
motor_left_time = 2.0
motor_right_time = 4.0
motor_speed = 0.5  

while vis.Run():
    current_time = sys.GetChTime()

    
    if current_time >= motor_left_time and current_time < motor_right_time:
        turtlebot.SetLeftMotorSpeed(motor_speed)
        turtlebot.SetRightMotorSpeed(-motor_speed)  
    elif current_time >= motor_right_time:
        turtlebot.SetLeftMotorSpeed(motor_speed)
        turtlebot.SetRightMotorSpeed(motor_speed)  

    else:
        turtlebot.SetLeftMotorSpeed(0)
        turtlebot.SetRightMotorSpeed(0)

    
    sys.DoStepDynamics(time_step)

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()