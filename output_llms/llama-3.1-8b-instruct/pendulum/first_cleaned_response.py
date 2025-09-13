import chrono as c
import chrono_irrlicht as irr


c.init()


ground = c.Body(c.ChronoMaterial(0.0, 0.0, 0.0), c.ChronoBodyData(0.0, 0.0, 0.0))
ground.SetPos(c.Vec3(0.0, 0.0, 0.0))


pendulum = c.Body(c.ChronoMaterial(1.0, 0.0, 0.0), c.ChronoBodyData(0.0, 0.0, 0.0))
pendulum.SetPos(c.Vec3(0.0, 1.0, 0.0))
pendulum.SetMass(1.0)
pendulum.SetInertia(c.Mat3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))


joint = c.RevoluteJoint(pendulum, ground, c.Vec3(0.0, 0.0, 0.0), c.Vec3(0.0, 0.0, 0.0))


pendulum.SetForce(c.Vec3(0.0, 0.0, 0.0))
pendulum.SetTorque(c.Vec3(0.0, 0.0, 0.0))


vis = irr.IrrlichtVis()


vis.AddBody(ground)
vis.AddBody(pendulum)
vis.AddJoint(joint)


sim = c.Simulation()
sim.AddBody(ground)
sim.AddBody(pendulum)
sim.AddJoint(joint)


log = c.Logger()
log.AddBody(pendulum)


while True:
    sim.Step()
    vis.Render()
    log.Log()
    c.wait(0.01)