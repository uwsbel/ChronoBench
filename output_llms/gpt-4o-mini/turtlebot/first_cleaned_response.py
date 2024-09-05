import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemNSC()


chrono.ChSystemNSC.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetMaterialSurface().SetFriction(0.5)
chrono.ChSystemNSC.Add(ground)


turtlebot = chrono.ChBody()
turtlebot.SetMass(1.0)  
turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))  
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
turtlebot.GetMaterialSurface().SetFriction(0.5)
chrono.ChSystemNSC.Add(turtlebot)


wheel_radius = 0.1
wheel_width = 0.05


left_wheel = chrono.ChBody()
left_wheel.SetMass(0.1)
left_wheel.SetPos(chrono.ChVectorD(-0.15, 0.1, 0))
left_wheel.GetMaterialSurface().SetFriction(0.5)
chrono.ChSystemNSC.Add(left_wheel)


right_wheel = chrono.ChBody()
right_wheel.SetMass(0.1)
right_wheel.SetPos(chrono.ChVectorD(0.15, 0.1, 0))
right_wheel.GetMaterialSurface().SetFriction(0.5)
chrono.ChSystemNSC.Add(right_wheel)


application = chronoirr.ChIrrApp(chrono.ChSystemNSC, "Turtlebot Simulation", chronoirr.dimension2d(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0, 1, 3), chrono.ChVectorD(0, 0.1, 0))


time_step = 0.01
current_time = 0.0
turn_left = False
turn_right = False

while application.GetDevice().run():
    
    chrono.ChSystemNSC.DoStepDynamics(time_step)
    
    
    if current_time < 2.0:
        
        left_wheel.SetPos(left_wheel.GetPos() + chrono.ChVectorD(0, 0, 0.1))
        right_wheel.SetPos(right_wheel.GetPos() + chrono.ChVectorD(0, 0, 0.1))
    elif 2.0 <= current_time < 4.0:
        
        left_wheel.SetPos(left_wheel.GetPos() + chrono.ChVectorD(0, 0, 0.05))
        right_wheel.SetPos(right_wheel.GetPos() + chrono.ChVectorD(0, 0, 0.15))
    elif 4.0 <= current_time < 6.0:
        
        left_wheel.SetPos(left_wheel.GetPos() + chrono.ChVectorD(0, 0, 0.15))
        right_wheel.SetPos(right_wheel.GetPos() + chrono.ChVectorD(0, 0, 0.05))
    else:
        
        left_wheel.SetPos(left_wheel.GetPos())
        right_wheel.SetPos(right_wheel.GetPos())

    
    current_time += time_step

    
    application.DrawAll()


application.GetDevice().drop()