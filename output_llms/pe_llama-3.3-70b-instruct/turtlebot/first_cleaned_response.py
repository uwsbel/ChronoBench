import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


sys.Set_G(chrono.ChVectorD(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, chrono.ChColor(0.5, 0.5, 0.5))
ground.SetPos(chrono.ChVectorD(0, -5, 0))
ground.SetBodyFixed(True)
sys.Add(ground)


turtlebot = veh.Turtlebot(sys)
turtlebot.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 5, 0), chrono.QUNIT))
turtlebot.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 10, 10))
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Turtlebot Simulation")
vis.AttachSystem(sys)
vis.Initialize()


dt = 0.01
t_end = 10
t = 0


left_motor_speed = 5
right_motor_speed = 5
left_motor_turn_time = 2
right_motor_turn_time = 4


while vis.Run() and t < t_end:
    
    sys.DoStepDynamics(dt)

    
    if t < left_motor_turn_time:
        turtlebot.SetMotorSpeed(left_motor_speed, right_motor_speed)
    elif t < right_motor_turn_time:
        turtlebot.SetMotorSpeed(-left_motor_speed, right_motor_speed)
    else:
        turtlebot.SetMotorSpeed(0, 0)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    t += dt

    
    print(f"Time: {t:.2f}")


vis.Close()