import chrono as ch
import chrono.irrlicht as chirr


ch.ChEngine.Initialize()


app = chirr.ChIrrApp(ch.ChSystem(), "Simple Pendulum", chirr.IrrlichtInfo())


ground = ch.ChBodyStatic()
ground.SetPos(ch.ChVector3d(0, 0, -10))


pendulum = ch.ChBody()
pendulum.SetMass(1.0)
pendulum.SetPos(ch.ChVector3d(0, 5, 0))
pendulum.SetRot(ch.ChQuaterniond(1, 0, 0, 0))


inertia = ch.ChMatrix33d(1.0)
inertia(0, 0) = 1.0  
inertia(1, 1) = 1.0  
inertia(2, 2) = 0.01  
pendulum.SetInertia(inertia)


joint = ch.ChRevolute()
joint.Init(ground, pendulum, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))


system = app.GetSystem()
system.Add(ground)
system.Add(pendulum)
system.Add(joint)


visualization = ch.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle("Simple Pendulum")
visualization.Initialize()


def LogPendulumState(t):
    print(f"Time: {t}, Pendulum Position: {pendulum.GetPos()}, Pendulum Velocity: {pendulum.GetPos_dt()}")


while visualization.Run():
    system.DoStepDynamics(0.01)
    LogPendulumState(system.GetChTime())

    
    visualization.BeginScene()
    visualization.EndScene()