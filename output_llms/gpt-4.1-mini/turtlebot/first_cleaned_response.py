import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math




system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




ground = chrono.ChBody()
ground.SetBodyFixed(True)

ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(10, 10, 0.1)  
ground.AddAsset(ground_shape)

ground_color = chrono.ChColorAsset()
ground_color.SetColor(chrono.ChColor(0.4, 0.25, 0))
ground.AddAsset(ground_color)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))  
system.Add(ground)





try:
    import pychrono.robot
except ImportError:
    raise ImportError("pychrono.robot module is required for Turtlebot but is not found.")


turtlebot = pychrono.robot.Turtlebot()

initial_pos = chrono.ChVectorD(0, 0, 0)  

initial_rot = chrono.ChQuaternionD()
initial_rot.Q_from_AngAxis(math.radians(90), chrono.ChVectorD(0,0,1))
turtlebot.SetPos(initial_pos)
turtlebot.SetRot(initial_rot)


turtlebot.GetSystem().Add(turtlebot.GetBody())







app = chronoirr.ChIrrApp(system, "Turtlebot on Terrain", chronoirr.dimension2du(1024,768))


app.AddTypicalSky()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(3, -5, 2), chronoirr.vector3df(0, 0, 0.2))  
app.AssetBindAll()
app.AssetUpdateAll()


time_step = 0.01  
app.SetTimestep(time_step)


turn_left_start = 2.0   
turn_left_end   = 4.0   
turn_right_start = 6.0  
turn_right_end   = 8.0  


motor_speed = 2.0  


def set_wheel_speeds(left_speed, right_speed):
    
    m_left = turtlebot.GetMotorLeft()
    m_right = turtlebot.GetMotorRight()
    
    m_left.SetSpeed(left_speed)
    m_right.SetSpeed(right_speed)


set_wheel_speeds(0, 0)


while app.GetDevice().run():
    current_time = system.GetChTime()

    
    if turn_left_start <= current_time < turn_left_end:
        
        set_wheel_speeds(motor_speed*0.5, motor_speed)
    elif turn_right_start <= current_time < turn_right_end:
        
        set_wheel_speeds(motor_speed, motor_speed*0.5)
    else:
        
        set_wheel_speeds(motor_speed, motor_speed)

    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    system.DoStepDynamics(time_step)