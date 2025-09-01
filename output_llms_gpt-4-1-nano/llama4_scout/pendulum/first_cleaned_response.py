import pychrono as chrono
import pychrono.irrlicht as irr


chrono.ChEngine.Initialize()


app = irr.ChIrrApp()


system = chrono.ChSystem()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)


pendulum = chrono.ChBody()
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
pendulum.SetPos(chrono.ChVectorD(1.0, 0.0, 0.0))
system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Init(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(joint)


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = irr.ChIrrApp(system, "Simple Pendulum", chrono.ChVectorD(800, 600), True)


vis.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vis.SetFrameRate(60)
vis.SetTimeStep(0.01)


def LogPendulumState(t):
    print(f"Time: {t:.2f} s")
    print(f"Pendulum position: {pendulum.GetPos()}")
    print(f"Pendulum velocity: {pendulum.GetPos_dt()}")


while vis.Run():
    vis.ComputeSimulationStep()
    LogPendulumState(vis.GetChTime())