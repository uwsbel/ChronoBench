import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irr
import numpy as np
import time as t
import math as m


def main():
    
    system = chrono.ChSystemNSC()

    
    csys = chrono.ChCollisionSystemSMM()
    csys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    ground_mat = chrono.ChContactMaterialNSC()
    ground = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, ground_mat)
    ground.SetPos(chrono.ChVector3d(0, 0, -1))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile(
        "textures/blue.png"), chrono.ChVisualShape.TexturingTextureCoords)

    system.Add(ground)

    
    turtlebot = robot.TurtleBot()
    turtlebot.SetPos(chrono.ChVector3d(0, -8, 1))
    turtlebot.SetRot(chrono.ChQuaterniond(1, 0, 0, -0.5 * m.pi))
    turtlebot.Initialize()
    system.Add(turtlebot)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Turtlebot Demo - Controlled by Python script')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 0, 1.5), 1.5, 1.5)
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 100), 100, 8, 40, 40)

    
    time_step = 2e-4
    time_end = 1000

    timer = chrono.ChRealtimeStepTimer()
    while (vis.Run() & (system.GetChTime() < time_end)):

        time = system.GetChTime()

        if time < 2:
            turtlebot.SetMotorsSpeeds(0, 0)
        elif time < 4:
            turtlebot.SetMotorsSpeeds(5, -5)
        elif time < 6:
            turtlebot.SetMotorsSpeeds(-5, 5)
        else:
            turtlebot.SetMotorsSpeeds(5, 5)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

        timer.Spin(time_step)

    print("Sim time:", system.GetChTime())






main()