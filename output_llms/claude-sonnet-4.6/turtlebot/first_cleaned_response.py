import pychrono as chrono
import pychrono.robot as turtlebot
try:
    import pychrono.irrlicht as chronoirr
except:
    print("Irrlicht module not available")
    exit()




system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)




ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.5)
ground_mat.SetRestitution(0.01)

ground = chrono.ChBodyEasyBox(
    20, 20, 0.1,   
    1000,          
    True,          
    True,          
    ground_mat     
)
ground.SetPos(chrono.ChVector3d(0, 0, -0.05))  
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)





init_pos = chrono.ChVector3d(0, 0, 0.2)   
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  

robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Simulation on Rigid Terrain")
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, -3, 2),   
    chrono.ChVector3d(0, 0, 0)     
)
vis.AddTypicalLights()
vis.AddLight(
    chrono.ChVector3d(3, -3, 3),   
    5,                              
    chrono.ChColor(0.8, 0.8, 0.8)  
)




time_step = 1e-3   
time_end  = 20.0   


FULL_SPEED   =  5.0   
LEFT_SPEED   =  2.0   
RIGHT_SPEED  = -2.0   
STOP         =  0.0


motor_schedule = [
    (0.0,  5.0,  FULL_SPEED,  FULL_SPEED),   
    (5.0,  8.0,  LEFT_SPEED,  FULL_SPEED),   
    (8.0, 13.0,  FULL_SPEED,  FULL_SPEED),   
    (13.0, 16.0, FULL_SPEED,  RIGHT_SPEED),  
    (16.0, 20.0, FULL_SPEED,  FULL_SPEED),   
]

def get_motor_speeds(t):
    
    for (t_start, t_end, left, right) in motor_schedule:
        if t_start <= t < t_end:
            return left, right
    return STOP, STOP




time = 0.0

print("Starting Turtlebot simulation...")
print("Controls:")
print("  0-5s:   Move straight")
print("  5-8s:   Turn left")
print("  8-13s:  Move straight")
print("  13-16s: Turn right")
print("  16-20s: Move straight")

while vis.Run() and time < time_end:
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    left_speed, right_speed = get_motor_speeds(time)

    
    
    robot.SetMotorSpeed(left_speed,  turtlebot.TurtleBot.LEFT)
    robot.SetMotorSpeed(right_speed, turtlebot.TurtleBot.RIGHT)

    
    robot.Update()

    
    system.DoStepDynamics(time_step)

    time += time_step

    
    if abs(time % 1.0) < time_step:
        pos = robot.GetChassisBody().GetPos()
        print(f"t={time:.1f}s | pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
              f"| motors: L={left_speed:.1f}, R={right_speed:.1f} rad/s")

print("Simulation complete.")