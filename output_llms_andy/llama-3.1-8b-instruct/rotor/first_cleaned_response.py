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
beam.SetCrossSectionArea(0.01)
beam.SetInertiaMomentX(0.0001)
beam.SetInertiaMomentY(0.0001)
beam.SetInertiaMomentZ(0.0001)
beam.SetMass(0.1)


flywheel = ch.ChBody()
flywheel.SetName("Flywheel")
flywheel.SetDensity(1.0)
flywheel.SetMass(0.1)
flywheel.SetInertiaMomentX(0.0001)
flywheel.SetInertiaMomentY(0.0001)
flywheel.SetInertiaMomentZ(0.0001)
flywheel.SetPosition(ch.Vec3(0, 0, 0.5))
flywheel.SetRotation(ch.Mat3(ch.PI/2, 0, 0))


motor = ch.ChMotor()
motor.SetName("Motor")
motor.SetType(ch.MOTOR_ROTATIONAL)
motor.SetMaxTorque(10)
motor.SetTargetPosition(10)
motor.SetTargetVelocity(10)


beam.AddPointMass(0.5, flywheel)


motor.AddPointMass(0, beam)


world.Add(beam)


world.Add(flywheel)


world.Add(motor)


world.SetFemVisualization(True)
world.SetIrrlichtVisualization(True)


world.SetTimestep(1e-4)
world.SetMaxStep(1e-3)
world.SetMaxTime(10)


world.Run()