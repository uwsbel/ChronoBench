import pychrono as chrono
import pychrono.irrlicht as irrlicht


system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)



Ixx = (1.0 / 12.0) * (1.0**2 + 0.1**2)  
Iyy = (1.0 / 12.0) * (0.1**2 + 0.1**2)
Izz = Ixx  
pendulum.SetInertiaXX(chrono.ChVectorD(Ixx, Iyy, Izz))


pendulum.SetPos(chrono.ChVectorD(0, -0.5, 0))


box = chrono.ChBoxShape()
box.GetBoxGeometry().Size = chrono.ChVectorD(0.05, 0.5, 0.05)  
pendulum.AddAsset(box)
pendulum.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.6, 0, 0)))  


ground_box = chrono.ChBoxShape()
ground_box.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
ground.AddAsset(ground_box)
ground.AddAsset(chrono.ChColorAsset(chrono.ChColor(0, 0, 0)))  


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.AddLink(joint)


application = irrlicht.ChIrrApp(system, 'Pendulum Simulation', irrlicht.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalCamera(irrlicht.vector3df(0, 0, 3), irrlicht.vector3df(0, 0, 0))
application.AddTypicalLight()

application.AssetBindAll()
application.AssetUpdateAll()


system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
step_size = 1e-3
log_interval = 0.1
current_log_time = 0.0

while application.GetDevice().run():
    time = system.GetChTime()
    
    
    if time >= current_log_time:
        angle = pendulum.GetRot().Q_to_Euler123().z  
        ang_vel = pendulum.GetWvel_loc().z  
        print(f"Time: {time:.3f}, Angle: {angle:.3f} rad, Angular velocity: {ang_vel:.3f} rad/s")
        current_log_time += log_interval
    
    
    system.DoStepDynamics(step_size)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()