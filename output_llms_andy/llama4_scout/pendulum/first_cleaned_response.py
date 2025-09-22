import chrono as ch
import chrono.irrlicht as irr


ch.CHGlobals.Initialize()


app = irr.IrrlichtApplication()


system = ch.ChSystem()


ground = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
ground.SetPos(ch.ChVector3d(0, -5, 0))
ground.SetBodyFixed(True)
system.Add(ground)


pendulum = ch.ChBodyEasyBox(0.1, 1, 0.1, 1, False, False)
pendulum.SetPos(ch.ChVector3d(0, 0, 0))
pendulum.SetRot(ch.ChQuaterniond(1, 0, 0, 0))
system.Add(pendulum)






joint = ch.ChRevoluteJoint(ground, pendulum, ch.ChVector3d(0, -5, 0), ch.ChVector3d(0, 0, 0))
joint.SetModulusOfElasticity(1e6)
joint.SetDissipation(0.5)
system.Add(joint)







vis = irr.IrrlichtSystem(system, app)
vis.SetCameraVertical(chrono.ChVector3d(0, -10, 0))
vis.SetCameraTarget(chrono.ChVector3d(0, 0, 0))


while vis.Run() and not app.HasToExit():
    system.DoStepDynamics(0.02)

    
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetPos_dt()
    print(f"Pendulum Position: {pendulum_pos}, Velocity: {pendulum_vel}")


del vis
del app