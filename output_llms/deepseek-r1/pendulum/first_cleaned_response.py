import pychrono as chrono
import pychrono.irrlicht as irrlicht
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
system.Add(ground)


length = 2.0    
radius = 0.05   
density = 1000  


pendulum = chrono.ChBodyEasyCylinder(radius, length, density, True, False)
pendulum.SetPos(chrono.ChVectorD(0, -length/2, 0))  
pendulum.SetRot(chrono.Q_from_AngZ(math.radians(30)))  
pendulum.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.8, 0.2))
system.Add(pendulum)


revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, 
                    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),  
                                     chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))  
system.Add(revolute)


vis = irrlicht.ChIrrApp(system, "Simple Pendulum", irrlicht.dimension2du(800, 600))
vis.AddTypicalLights()
vis.AddTypicalCamera(irrlicht.vector3df(0, -1, 3), irrlicht.vector3df(0, 0, 0))
vis.AssetBindAll()
vis.AssetUpdateAll()


time_step = 0.01
log_interval = 0.1
next_log_time = 0

while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.DoStepDynamics(time_step)
    vis.EndScene()
    
    current_time = system.GetChTime()
    
    
    if current_time >= next_log_time:
        
        angle = math.degrees(revolute.GetRelAngle())
        ang_vel = revolute.GetRelWvel().z
        
        
        local_end = chrono.ChVectorD(0, -length/2, 0)  
        global_end = pendulum.TransformPointLocalToParent(local_end)
        
        print(f"Time: {current_time:.2f}s | "
              f"Angle: {angle:.2f}° | "
              f"Angular Velocity: {ang_vel:.2f} rad/s | "
              f"End Position: ({global_end.x:.2f}, {global_end.y:.2f}, {global_end.z:.2f})")
        
        next_log_time += log_interval