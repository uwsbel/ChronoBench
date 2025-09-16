import chrono as ch
import numpy as np


ch.InitChSystem()


world = ch.ChWorld()


beam = ch.ChBeam()
beam.SetName("JeffcottRotor")
beam.SetDensity(1.0)
beam.SetYoungModulus(200e9)
beam.SetPoissonRatio(0.3)
beam.SetLength(1.0)
beam.SetRadius(0.05)
beam.SetNumElements(10)
world.Add(beam)


flywheel = ch.ChBody()
flywheel.SetName("Flywheel")
flywheel.SetDensity(1.0)
flywheel.SetMass(1.0)
flywheel.SetInertia(0.5, 0.5, 0.5)
flywheel.SetPos(ch.Vec(0, 0, 0.5))
flywheel.SetAng(ch.Vec(0, 0, 0))
world.Add(flywheel)


flywheel.SetPos(ch.Vec(0, 0, 0.5))
flywheel.SetAng(ch.Vec(0, 0, 0))
beam.SetPos(ch.Vec(0, 0, 0))
beam.SetAng(ch.Vec(0, 0, 0))
ch.ChAttachCollide(flywheel, beam, ch.Vec(0, 0, 0), ch.Vec(0, 0, 0))


motor = ch.ChMotor()
motor.SetName("Motor")
motor.SetType(ch.ChMotorType.ROTATIONAL)
motor.SetMaxTorque(10.0)
motor.SetPos(ch.Vec(0, 0, 0))
motor.SetAng(ch.Vec(0, 0, 0))
motor.SetAxis(ch.Vec(0, 0, 1))
beam.SetPos(ch.Vec(0, 0, 0))
beam.SetAng(ch.Vec(0, 0, 0))
ch.ChAttachCollide(motor, beam, ch.Vec(0, 0, 0), ch.Vec(0, 0, 0))


beam.SetPos(ch.Vec(0, 0, 0))
beam.SetAng(ch.Vec(0, 0, 0))
flywheel.SetPos(ch.Vec(0, 0, 0.5))
flywheel.SetAng(ch.Vec(0, 0, 0))
motor.SetPos(ch.Vec(0, 0, 0))
motor.SetAng(ch.Vec(0, 0, 0))


t = 0
dt = 0.001
t_end = 10
while t < t_end:
    world.DoStepDynamics(dt)
    t += dt


ch.ChFemVisualize(world, 1)
ch.ChIrrVisualize(world, 1)