import chrono as ch
import chrono.irrlicht as irr
import chrono.postprocess as post


ch.ChEngine.Initialize()


system = ch.ChSystemDofs()


system.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))


beam = ch.ChBeamIGA()
beam.SetPos(ch.ChVector3d(0, 0, 0))
beam.SetRot(ch.ChQuaterniond(1, 0, 0, 0))
beam.SetLength(1.0)
beam.SetNumElements(10)
beam.SetSection(ch.ChBeamSectionIGA(ch.ChVector3d(0.1, 0.1, 0.1), 7850, 2e8, 0.3))
beam.AddSectionMass(10)


system.Add(beam)


flywheel = ch.ChBody()
flywheel.SetMass(10)
flywheel.SetInertiaXX(ch.ChVector3d(1, 1, 1))
flywheel.SetPos(ch.ChVector3d(0.5, 0, 0))


system.Add(flywheel)


link = ch.ChLinkLockPrizmaticPrizmatic()
link.SetName('link')
link.SetBody1(beam.GetBody())
link.SetBody2(flywheel)
link.SetFrame1Pnt(ch.ChVector3d(0, 0, 0))
link.SetFrame2Pnt(ch.ChVector3d(0, 0, 0))
system.Add(link)


motor = ch.ChLinkMotorRotation()
motor.SetName('motor')
motor.SetBody1(beam.GetBody())
motor.SetBody2(None)  
motor.SetModulusOfTwist(1e4)
motor.SetMotorFunction(ch.ChFunction_Sine(0, 1, 0))


system.Add(motor)


app = irr.ChIrrApp(system, 'Jeffcott Rotor Simulation', 800, 600)


app.SetFEMVisible(True)


app.AssetBindAll()
app.AssetUpdateAll()
app.Run()