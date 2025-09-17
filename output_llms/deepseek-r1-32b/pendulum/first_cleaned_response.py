importchrono
import chronoirr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


application = chronoirr.ChIrrApp(system, "Simple Pendulum", chronoirr.dimension2du(800, 600))
application.AddTypicalIrrlichtLight()
application.AddTypicalIrrlichtCamera(chrono.ChVectorD(0, 5, 5), chrono.ChVectorD(0, 0, 0))


ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetMass(0)  
ground.GetCollisionModel().AddBox(chrono.ChCollisionModel(), chrono.ChVectorD(10, 1, 10))
ground.GetVisualShape().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.5, 0.5, 0.5)))
system.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetPos(chrono.ChVectorD(0, 2, 0))
pendulum.SetMass(1.0)  
pendulum.GetInertia().SetIdentity(0.1)  
pendulum.GetCollisionModel().AddSphere(chrono.ChCollisionModel(), 0.2)
pendulum.GetVisualShape().SetMaterial(chrono.ChVisualMaterial(chrono.ChColor(0.8, 0, 0)))
system.Add(pendulum)


joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(
    chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 0, 1))))
joint.SetAngleX(45 * chrono.CH_C_DEG_TO_RAD)  
system.Add(joint)


pendulum.GetVisualShape().AddSphere(0.2, chrono.ChColor(0.8, 0, 0))


constraint = chrono.ChLinkLockPointToLine()
constraint.Initialize(ground, pendulum, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(constraint)


time_step = 0.001
total_time = 10.0


application.Begin()
while application.GetDevice().Run():
    time = system.GetChTime()
    
    if time >= total_time:
        break
        
    
    system.DoStepDynamics(time_step)
    
    
    application.Render()
    
    
    if time % 0.1 == 0:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {time:.2f}s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")
        
    
    if application.GetDevice().IsKeyDown(chronoirr.KEY_ESCAPE):
        break

application.End()