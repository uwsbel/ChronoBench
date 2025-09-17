import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./chrono_data/')


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)


tb = veh.Turtlebot(mysystem, chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(chrono.CH_C_PI / 2))


tb.SetMotorType(veh.Turtlebot.MOTOR_TYPE_POSITION)
tb.SetMotorPosition(chrono.CH_C_PI / 4)


vis = chronoirr.ChVisualSystemIrrlicht(mysystem)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 2, -2), chrono.ChVectorD(0, 0, 0))
vis.SetLogoTexture('./data/logo.png')
vis.SetLogoPosition(chrono.ChVectorD(10, 10))
vis.SetLogoScale(chrono.ChVectorD(0.5, 0.5))


stept = 0.01
tend = 10
while mysystem.GetChTime() < tend:
    
    if mysystem.GetChTime() < 2:
        tb.SetMotorSpeed(1, 1)
        tb.SetMotorSpeed(2, -1)
    elif mysystem.GetChTime() < 4:
        tb.SetMotorSpeed(1, -1)
        tb.SetMotorSpeed(2, 1)
    else:
        tb.SetMotorSpeed(1, 0)
        tb.SetMotorSpeed(2, 0)

    
    mysystem.DoStepDynamics(stept)
    vis.Render()


vis.Run()