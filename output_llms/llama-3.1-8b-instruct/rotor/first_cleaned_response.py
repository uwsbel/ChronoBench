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
flywheel.SetAng(0, 0, 0)
world.Add(flywheel)


motor = ch.ChMotor()
motor.SetName("Motor")
motor.SetType(ch.ChMotorType.ROTATIONAL)
motor.SetAxis(ch.Vec(1, 0, 0))
motor.SetSpeed(100)
motor.SetMaxForce(100)
motor.SetPos(ch.Vec(0, 0, 0))
motor.SetAng(0, 0, 0)
world.Add(motor)


flywheelAttach = ch.ChAttachMode()
flywheelAttach.SetBodyA(beam)
flywheelAttach.SetBodyB(flywheel)
flywheelAttach.SetPos(ch.Vec(0, 0, 0))
flywheelAttach.SetAng(0, 0, 0)
flywheelAttach.SetType(ch.ChAttachMode.FIXED)
world.Add(flywheelAttach)


beam.SetFemVisualization(True)
beam.SetFemColor(ch.ChColor(1, 0, 0))


irrlicht = ch.ChIrrAppSimple(world, "Jeffcott Rotor Simulation", ch.dimension2du(800, 600))
irrlicht.AddFrameScene(world)


while not irrlicht.KeyCloseDown():
    world.DoStep()
    irrlicht.BeginScene()
    irrlicht.DrawAll()
    irrlicht.EndScene()
    irrlicht.ShowImage()


ch.FinalCleanUp()