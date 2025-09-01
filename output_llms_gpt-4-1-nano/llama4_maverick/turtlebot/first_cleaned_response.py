import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('/path/to/chrono/data/')  
veh.SetDataPath('/path/to/chrono/data/vehicle/')  


sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  


ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.5))
sys.Add(ground)


init_pos = chrono.ChVectorD(0, 0, 0.2)  
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  


turtlebot_mat = chrono.ChMaterialSurfaceNSC()
turtlebot_body = chrono.ChBodyEasyBox(0.3, 0.3, 0.1, 1000, True, True, turtlebot_mat)
turtlebot_body.SetPos(init_pos)
turtlebot_body.SetRot(init_rot)
sys.Add(turtlebot_body)


wheel_mat = chrono.ChMaterialSurfaceNSC()
left_wheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 1000, True, True, wheel_mat)
left_wheel.SetPos(init_pos + chrono.ChVectorD(-0.1, 0, -0.1))
sys.Add(left_wheel)

right_wheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 1000, True, True, wheel_mat)
right_wheel.SetPos(init_pos + chrono.ChVectorD(0.1, 0, -0.1))
sys.Add(right_wheel)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Turtlebot Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1.5, 1.5, 1.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 1.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 2, 10, 40, 512)


time_step = 0.01
time_end = 10
motor_control_time = 0

while vis.Run() and sys.GetChTime() < time_end:
    sys.DoStepDynamics(time_step)
    
    
    current_time = sys.GetChTime()
    if current_time > motor_control_time:
        motor_control_time += 1  
        
        
        if int(current_time) % 2 == 0:
            
            left_wheel.SetPos_dt(chrono.ChVectorD(0, -1, 0))  
            right_wheel.SetPos_dt(chrono.ChVectorD(0, 1, 0))
        else:
            
            left_wheel.SetPos_dt(chrono.ChVectorD(0, 1, 0))
            right_wheel.SetPos_dt(chrono.ChVectorD(0, -1, 0))
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

vis.GetDevice().closeDevice()