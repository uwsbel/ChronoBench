import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea as fea
import numpy as np


chrono.ChEngine.Initialize()


system = chrono.ChSystem()


system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


app = irr.ChIrrApp(system, "Jeffcott Rotor Simulation", chrono.ChVectorD(800, 600))
app.SetTimestep(0.01)
app.SetTryRewind(True)


beam = fea.ChIGAbeam()
beam.SetLength(1.0)  
beam.SetSectionWidth(0.05)  
beam.SetSectionHeight(0.05)  
beam.SetNg(4)  
beam.SetNurbsOrder(3)  
beam.SetMaterial(chrono.ChMaterialShellGiant(200e9, 0.3, 7850))  


beam.SetStartPoint(chrono.ChVectorD(0, 0, 0))
beam.SetEndPoint(chrono.ChVectorD(1, 0, 0))


system.Add(beam)


flywheel = chrono.ChBody()
flywheel.SetMass(10)  
flywheel.SetInertia(chrono.ChMatrix33D(1, 0, 0, 0, 1, 0, 0, 0, 1))  


flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))
flywheel.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


system.Add(flywheel)


constraint = chrono.ChLinkLockPointLine(flywheel, beam)
constraint.SetPointA(chrono.ChVectorD(0, 0, 0))
constraint.SetPointB(chrono.ChVectorD(0.5, 0, 0))
system.Add(constraint)


motor = chrono.ChLinkMotorRotationAngle()
motor.SetMotorFunction(chrono.ChFunction_Sine(0, 1, 0))  


motor.SetPointA(chrono.ChVectorD(0, 0, 0))
motor.SetPointB(chrono.ChVectorD(0, 0, 0))


system.Add(motor)


motor.Connect(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))


fem_vis = fea.ChFEMvisualization(beam)
fem_vis.SetFEMdata(beam.GetNodes(), beam.GetElements())
app.Add(fem_vis)


irr_window = irr.ChIrrlicht irr_window(app)
irr_window.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))


while app.GetShouldKeepRunning():
    app.DoStep()
    irr_window.Update()