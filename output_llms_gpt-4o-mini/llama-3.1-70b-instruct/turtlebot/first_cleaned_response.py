import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)


turtlebot = veh.ChTurtlebot(mysystem, chrono.ChVectorD(0, 0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0))
turtlebot.SetMass(10)
turtlebot.SetInertia(chrono.ChVectorD(1, 1, 1))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Turtlebot Simulation')
vis.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
vis.AddTypicalLights()


vis.Add(ground)
vis.Add(turtlebot)


dt = 0.01
t_end = 10


t = 0
while t < t_end:
    
    if t < 2:
        turtlebot.SetMotorTorque(chrono.ChVectorD(1, 0, 0))  
    elif t < 4:
        turtlebot.SetMotorTorque(chrono.ChVectorD(-1, 0, 0))  
    else:
        turtlebot.SetMotorTorque(chrono.ChVectorD(0, 0, 0))  

    
    mysystem.DoStepDynamics(dt)

    
    vis.Update()
    vis.Render()

    
    t += dt

    
    print(f'Time: {t:.2f}')


vis.Run()