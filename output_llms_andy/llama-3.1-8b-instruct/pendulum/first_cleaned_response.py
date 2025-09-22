import chrono as ch
import chrono.irrlicht as irr


ch.InitChWorld()
irr.InitIrrlicht()


ground = ch.ChBodyEasyBox(10, 0.1, 0.1, 1000, 0.1, 0.1, 0)
ground.SetPos(ch.Vec(0, 0, 0))
ground.SetRot(ch.Vec(0, 0, 0))
ground.SetBodyFixed(True)


pendulum = ch.ChBodyEasyCylinder(0.1, 1, 10, 100, 0.1, 0.1, 0)
pendulum.SetPos(ch.Vec(0, 0, 1))
pendulum.SetRot(ch.Vec(0, 0, 0))
pendulum.SetMass(1)
pendulum.SetInertia(ch.Moments(0.01, 0, 0))


joint = ch.ChLinkRevolute(ground, pendulum)
joint.SetAnchor1(ch.Vec(0, 0, 0))
joint.SetAnchor2(ch.Vec(0, 0, 1))
joint.SetAxis(ch.Vec(0, 1, 0))
joint.SetLoch(0)
joint.SetHich(0)
joint.SetFriction(0)
joint.SetDamping(0)


irr.AddVisualization(ch.GetWorld(), "Pendulum Simulation")
ch.AddLog(ch.GetWorld(), "Pendulum Position: ", pendulum.GetPos())
ch.AddLog(ch.GetWorld(), "Pendulum Velocity: ", pendulum.GetV())
ch.SetLogPeriod(0.01)


ch.RunSimulation(10)


ch.DestroyChWorld()
irr.DestroyIrrlicht()