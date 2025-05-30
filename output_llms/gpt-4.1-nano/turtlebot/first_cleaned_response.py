import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)  
ground.SetPosition(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
system.Add(ground)






chassis = chrono.ChBodyEasyBox(0.3, 0.1, 0.2, 1000, True, True)
chassis.SetPos(chrono.ChVectorD(0, 0.2, 0))
system.Add(chassis)


left_wheel = chrono.ChBodyEasyCylinder(0.05, 0.02, 100, True, True)
left_wheel.SetPos(chrono.ChVectorD(-0.1, 0.05, 0.1))
system.Add(left_wheel)


right_wheel = chrono.ChBodyEasyCylinder(0.05, 0.02, 100, True, True)
right_wheel.SetPos(chrono.ChVectorD(0.1, 0.05, 0.1))
system.Add(right_wheel)


left_joint = chrono.ChLinkLockRevolute()
left_joint.SetConstrainedBody(chassis, left_wheel)
left_joint.SetAxis(chrono.ChVectorD(0, 1, 0))
left_joint.Initialize(chassis, left_wheel, chrono.C weißen npaffiliationll
system.Add(left_joint)

right_joint = chrono.ChLinkLockRevolute()
right_joint.SetConstrainedBody(chassis, right_wheel)
right_joint.SetAxis(chrono.ChVectorD(0, 1, 0))
right_joint.Initialize(chassis, right_wheel, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(right_joint)


application = irr.ChIrrApp(system, "Turtlebot Simulation", irr.dimension2du(800, 600))
application.AddLights()
application.SetCamera(irr.QCAMERA_TYPE::CAMERA_PREVIEW, 0.5, 1.0, -1.5)
application.SetAmbientLight(irr.SColorf(0.5, 0.5, 0.5))
application.AssetBindAll()
application.AssetUpdateAll()


def set_wheel_speed(wheel, speed):
    
    torque = 0.1 * speed  
    wheel.GetTractionConstraint().SetAppliedForce(chrono.VECTORS_TO_FROM_VECTOR(chrono.ChVectorD(torque, 0, 0)))


time = 0
dt = 0.01
end_time = 20

while application.GetDevice().run():
    
    if 2 <= time < 5:
        
        set_wheel_speed(left_wheel, 1.0)   
        set_wheel_speed(right_wheel, -1.0)  
    elif 10 <= time < 13:
        
        set_wheel_speed(left_wheel, -1.0)
        set_wheel_speed(right_wheel, 1.0)
    else:
        
        set_wheel_speed(left_wheel, 1.0)
        set_wheel_speed(right_wheel, 1.0)

    system.DoStepDynamics(dt)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    application.GetDevice().run()
    time += dt