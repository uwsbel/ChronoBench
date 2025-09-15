import pychrono as chrono
import pychrono.irrlicht as irr
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
pendulum.SetMass(1.0)
pendulum.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)))  


inertia = chrono.ChMatrix33D()
inertia.SetElement(1, 1, (1/12) * 1.0 * (2.0**2))  
pendulum.SetInertia(inertia)
system.Add(pendulum)


pendulum_shape = chrono.ChBoxShape()
pendulum_shape.GetBoxGeometry().Size = chrono.ChVectorD(1, 0.05, 0.05)  
pendulum.AddVisualShape(pendulum_shape, chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)))


revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum, 
                    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                     chrono.Q_from_AngZ(0)))
system.Add(revolute)


pendulum.SetRot(chrono.Q_from_AngZ(math.radians(30)))


application = irr.ChIrrApp(system, "Simple Pendulum", 800, 600)
application.AddTypicalCamera(irr.vector3df(0, 2, 3))  
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()


time_step = 0.01
log_interval = 0.1
next_log_time = 0

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStepDynamics(time_step)
    application.EndScene()
    
    current_time = system.GetChTime()
    if current_time >= next_log_time:
        
        cog_pos = pendulum.GetFrame_COG_to_abs().GetPos()
        cog_vel = pendulum.GetFrame_COG_to_abs().GetPos_dt()
        print(f"Time: {current_time:.2f}s, Position: ({cog_pos.x:.2f}, {cog_pos.y:.2f}, {cog_pos.z:.2f})m, Velocity: ({cog_vel.x:.2f}, {cog_vel.y:.2f}, {cog_vel.z:.2f})m/s")
        next_log_time += log_interval