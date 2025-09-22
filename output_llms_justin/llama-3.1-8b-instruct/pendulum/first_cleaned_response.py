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


joint = ch.ChRevoluteJoint()
joint.SetPos(ch.Vec(0, 0, 0))
joint.SetAxis(ch.Vec(0, 0, 1))
joint.SetBodyA(ground)
joint.SetBodyB(pendulum)


ch.RegisterBody(ground)
ch.RegisterBody(pendulum)
ch.RegisterJoint(joint)


vis = irr.CreateVisualization()
vis.Add(ch.GetScene())


step = ch.ChStepSettings()
step.SetStepType(ch.CH_STEP_TYPE_TIME)
step.SetStepInTime(0)
step.SetStepOutTime(10)
step.SetStepTime(0.01)
step.SetStepMax(10000)
step.SetTolerance(1e-6)
step.SetMaxIter(10)


log_file = 'pendulum.log'
log_freq = 0.1  


ch.RunSimulation(vis, step, log_file, log_freq)