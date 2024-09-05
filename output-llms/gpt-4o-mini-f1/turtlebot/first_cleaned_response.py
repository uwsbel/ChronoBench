import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as irr
import math


sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(40, 40, 2, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))  
ground.SetFixed(True)  
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  
sys.Add(ground)  


robot = turtlebot.TurtleBot(sys)
init_pos = chrono.ChVector3d(0, 0, 0)  
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  
robot.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))  
robot.SetChassisFixed(False)  
robot.SetVerbose(False)  
robot.SetContactMethod(chrono.ChContactMethod_NSC)  
robot.SetTireType(turtlebot.TireModelType_TMEASY)  
robot.SetTreadType(turtlebot.TireTreadType_RUBBER)  
robot.SetTrackWidth(0.6)  
robot.SetWheelRadius(0.3)  
robot.SetWheelMass(1)  
robot.SetChassisBodyFixed(False)  
robot.Initialize()  


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(irr.CAMERA_VERT_Z)  
vis.SetWindowSize(1280, 720)  
vis.SetWindowTitle('Turtlebot Rigid Terrain')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  
vis.AddTypicalLights()  


time = 0  
while vis.Run():
    
    if time < 2.0:
        robot.SetMotorSpeed(0, 0)  
    elif time < 4.0:
        robot.SetMotorSpeed(0.5, 0.5)  
    elif time < 6.0:
        robot.SetMotorSpeed(0.5, -0.5)  
    elif time < 8.0:
        robot.SetMotorSpeed(-0.5, -0.5)  
    else:
        robot.SetMotorSpeed(-0.5, 0.5)  

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(1e-3)
    time += 1e-3