import math
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr




system = chrono.ChSystemNSC()


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))            
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)




ground_mat = chrono.ChContactMaterialNSC()
ground      = chrono.ChBodyEasyBox(20, 20, 1,           
                                   1000,                
                                   True, True, ground_mat)

ground.SetPos(chrono.ChVectorD(0, 0, -0.6))              
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)




init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot    = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()


LEFT_DRIVE_WHEEL  = 0
RIGHT_DRIVE_WHEEL = 1




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chronoirr.CameraVerticalDir_Z)      
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot – Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1.5, 0.2),
              chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5),
                       chrono.ChVectorD(0, 0, 0.5),
                       3, 4, 10, 40, 512)






def move(mode: str):
    

    if mode == 'straight':
        
        robot.SetMotorSpeed( 1.0 * math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed( 1.0 * math.pi, RIGHT_DRIVE_WHEEL)

    elif mode == 'left':
        
        robot.SetMotorSpeed( 0.0,           LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-1.0 * math.pi, RIGHT_DRIVE_WHEEL)

    elif mode == 'right':
        
        robot.SetMotorSpeed(-1.0 * math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed( 0.0,           RIGHT_DRIVE_WHEEL)

    else:
        raise ValueError("Unknown mode '{}'. Use: straight | left | right"
                         .format(mode))





time_step  = 2.0e-3
time       = 0.0
current_mode = None      

while vis.Run():

    
    if time < 5.0:
        desired_mode = 'straight'
    elif time < 10.0:
        desired_mode = 'left'
    else:
        desired_mode = 'right'

    
    if desired_mode != current_mode:
        move(desired_mode)
        print("[{:.2f} s] robot is now moving {}".format(time, desired_mode))
        current_mode = desired_mode

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)
    time += time_step