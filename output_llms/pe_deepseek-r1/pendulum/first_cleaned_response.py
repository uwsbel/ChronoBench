import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

def main():
    
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, -9.81, 0))  
    
    
    ground = chrono.ChBody()
    ground.SetFixed(True)  
    system.Add(ground)
    
    
    ground_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
    ground_shape.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    ground.AddVisualShape(ground_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    
    
    pendulum = chrono.ChBody()
    pendulum.SetMass(1.0)  
    pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))  
    
    
    pendulum_length = 2.0
    com_position = chrono.ChVector3d(0, -pendulum_length/2, 0)
    pendulum.SetPos(com_position)
    
    
    rod_shape = chrono.ChVisualShapeCylinder(0.05, pendulum_length)
    rod_shape.SetColor(chrono.ChColor(0.6, 0.6, 0.8))
    pendulum.AddVisualShape(rod_shape, chrono.ChFramed(chrono.ChVector3d(0, pendulum_length/2, 0)))
    
    sphere_shape = chrono.ChVisualShapeSphere(0.2)
    sphere_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
    pendulum.AddVisualShape(sphere_shape, chrono.ChFramed(chrono.ChVector3d(0, -pendulum_length, 0)))
    
    system.Add(pendulum)
    
    
    pivot = chrono.ChVector3d(0, 0, 0)
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(ground, pendulum, 
                    chrono.ChFramed(pivot, chrono.QuatFromAngleX(0)))
    system.Add(joint)
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Simple Pendulum Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, -1, 3))  
    vis.AddTypicalLights()
    
    
    time_step = 0.01
    frame_interval = 0.02  
    next_log_time = 0
    
    while vis.Run():
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        system.DoStepDynamics(time_step)
        
        
        current_time = system.GetChTime()
        if current_time >= next_log_time:
            
            pos = pendulum.GetPos()
            vel = pendulum.GetPosDt()
            
            
            theta = math.atan2(pos.x, -pos.y)  
            
            print(f"Time: {current_time:.3f}s | "
                  f"Position: ({pos.x:.3f}, {pos.y:.3f}) | "
                  f"Velocity: ({vel.x:.3f}, {vel.y:.3f}) | "
                  f"Angle: {math.degrees(theta):.2f}°")
            
            next_log_time += frame_interval

if __name__ == '__main__':
    main()