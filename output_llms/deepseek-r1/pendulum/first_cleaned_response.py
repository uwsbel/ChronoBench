import pychrono as chrono
import pychrono.irrlicht as irrlicht
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)


ground_box = chrono.ChBoxShape()
ground_box.GetBoxGeometry().Size = chrono.ChVector3d(0.5, 0.5, 0.5)
ground.AddVisualShape(ground_box)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 4.0/3.0))  
pendulum.SetPos(chrono.ChVector3d(0, -1, 0))
pendulum.SetRot(chrono.Q_from_AngZ(math.pi/6))  
system.Add(pendulum)


pendulum_shape = chrono.ChBoxShape()
pendulum_shape.GetBoxGeometry().Size = chrono.ChVector3d(0.2, 2.0, 0.2)
pend_shape_asset = pendulum.AddVisualShape(pendulum_shape)
pend_shape_asset.SetColor(chrono.ChColor(1.0, 0.0, 0.0))  


revolute = chrono.ChLinkRevolute()
revolute.Initialize(ground, pendulum,
    chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),  
    chrono.ChCoordsysD(chrono.ChVector3d(0, 1, 0), chrono.QUNIT))  
system.Add(revolute)


vis = irrlicht.ChIrrApp(system, "Simple Pendulum", 800, 600)
vis.AddTypicalCamera(chrono.ChVector3d(0, 0, 5))
vis.AddTypicalLights()
vis.AssetBindAll()
vis.AssetUpdateAll()
vis.Initialize()


time_step = 0.001
end_time = 10.0
log_interval = 0.1
last_log_time = -log_interval


while vis.GetDevice().run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    current_time = system.GetChTime()
    system.DoStepDynamics(time_step)

    
    if current_time - last_log_time >= log_interval:
        pos = pendulum.GetPos()
        vel = pendulum.GetLinVel()
        print(f"Time: {current_time:.2f}s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")
        last_log_time = current_time
    
    if current_time >= end_time:
        break