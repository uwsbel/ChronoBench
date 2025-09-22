import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




ground = chrono.ChBodyEasyBox(5, 0.2, 5, 1000, True, True)   
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))                  
ground.SetBodyFixed(True)
system.Add(ground)


ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/cubetexture_plywood.jpg"))
ground.GetAssets().push_back(ground_texture)





wheel_radius   = 0.1      
wheel_thick    = 0.05     
wheel_mass     = 1.0
chassis_size   = chrono.ChVectorD(0.4, 0.05, 0.3) 
chassis_mass   = 5.0
axle_y         = wheel_radius                       

start_pos      = chrono.ChVectorD(0, axle_y + chassis_size.y, 0)


chassis = chrono.ChBodyEasyBox(chassis_size.x*2,   
                               chassis_size.y*2,
                               chassis_size.z*2,
                               chassis_mass,
                               True, True)
chassis.SetPos(start_pos)
system.Add(chassis)


def make_wheel(offset_z, name="wheel"):
    wheel = chrono.ChBodyEasyCylinder(wheel_radius,  
                                      wheel_thick,   
                                      wheel_mass,
                                      True, True)
    wheel.SetRot(chrono.Q_from_AngAxis(math.pi / 2, chrono.ChVectorD(0, 0, 1)))  
    wheel.SetPos(start_pos + chrono.ChVectorD(0, 0, offset_z))
    wheel.SetCollide(True)
    system.Add(wheel)
    return wheel

wheel_L = make_wheel(+chassis_size.z, "wheel_L")  
wheel_R = make_wheel(-chassis_size.z, "wheel_R")  



rpm2rad = 2.0 * math.pi / 60.0

def add_motor(wheel, pos, direction):
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(wheel,          
                     chassis,        
                     chrono.ChFrameD(pos,            
                                     chrono.Q_from_AngAxis(
                                         math.pi/2, chrono.ChVectorD(0, 0, 1))))  
    motor.SetSpindleConstraint(True)        
    system.AddLink(motor)
    return motor

motor_L = add_motor(wheel_L, start_pos + chrono.ChVectorD(0, 0, +chassis_size.z), chrono.ChVectorD(1,0,0))
motor_R = add_motor(wheel_R, start_pos + chrono.ChVectorD(0, 0, -chassis_size.z), chrono.ChVectorD(1,0,0))


def set_speed(motor, omega):
    fun = chrono.ChFunction_Const(omega)
    motor.SetSpeedFunction(fun)


straight_omega = 10 * rpm2rad
set_speed(motor_L, straight_omega)
set_speed(motor_R, straight_omega)




app = chronoirr.ChIrrApp(system,
                         "PyChrono Turtlebot demo",
                         chronoirr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddLightWithShadow(chrono.ChVectorD(2, 4, 2),
                       chrono.ChVectorD(0, 0, 0),
                       3, 1, 7, 40)
app.AddCamera(chrono.ChVectorD(1, 0.7, 1),      
              chrono.ChVectorD(0, 0.2, 0))      
app.AssetBindAll()
app.AssetUpdateAll()


realtime_timer = chrono.ChRealtimeStepTimer()




step_size = 1.0 / 200.0    

print("Simulation running. ESC to quit.")

while app.GetDevice().run():
    
    t = system.GetChTime()

    if t < 3.0:
        
        set_speed(motor_L, straight_omega)
        set_speed(motor_R, straight_omega)

    elif t < 6.0:
        
        set_speed(motor_L, 5 * rpm2rad)
        set_speed(motor_R, 15 * rpm2rad)

    elif t < 9.0:
        
        set_speed(motor_L, 15 * rpm2rad)
        set_speed(motor_R, 5 * rpm2rad)

    else:
        
        set_speed(motor_L, 0.0)
        set_speed(motor_R, 0.0)

    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    
    system.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size)