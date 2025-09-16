import chrono as ch
import chrono.irrlicht as irr


ch.InitCh('pychrono_001')


ground = ch.ChBodyEasyBox(1, 1, 0.01, 0, 0, 0, ch.CH_COLOR_GRAY)
ground.SetPos(ch.Vec(0, 0, 0))
ground.SetRot(ch.Mat33(ch.PI/2, 0, 0))
ground.SetBodyType(ch.CH_BODY_TYPE_STATIC)
ground.SetMass(0)


pendulum = ch.ChBodyEasyCylinder(0.1, 0.2, 0.01, 0, 0, 0, ch.CH_COLOR_RED)
pendulum.SetPos(ch.Vec(0, 1, 0))
pendulum.SetRot(ch.Mat33(0, 0, 0))
pendulum.SetMass(1)
pendulum.SetInertiaTensor(0.05, 0, 0, 0, 0.05, 0, 0, 0, 0.05)
pendulum.SetBodyType(ch.CH_BODY_TYPE_DYNAMIC)


joint = ch.ChRevoluteJoint(ground, pendulum, ch.Vec(0, 1, 0), ch.Vec(0, 0, 0))
joint.SetAnchor(ch.Vec(0, 1, 0))
joint.SetAxis(ch.Vec(0, 0, 1))


vis = irr.ChIrrAppCreate(ch.GetEngine(), 640, 480, False, "Simple Pendulum Simulation")


log = ch.ChLogSetLevel(ch.LOG_LEVEL_INFO)
log.AddStream(ch.ChLogConsole())
log.AddStream(ch.ChLogFile('simple_pendulum.log'))


t = 0
dt = 0.01
while True:
    
    ch.StepSimulation(dt)

    
    pos = pendulum.GetPos()
    vel = pendulum.GetVel()

    
    log.LogInfo('Pendulum position: (%.4f, %.4f, %.4f)', pos.x, pos.y, pos.z)
    log.LogInfo('Pendulum velocity: (%.4f, %.4f, %.4f)', vel.x, vel.y, vel.z)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    t += dt