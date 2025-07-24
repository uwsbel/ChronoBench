import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import pychrono.iga as iga
import math
import os







system = chrono.ChSystemNSC()


beam = iga.ChBeamIGA()
beam.SetDataFile(chrono.GetChronoDataFile('iga/beam.json'))
beam.SetUniformSection()
beam.SetFixFirstDof()
beam.Initialize()
system.Add(beam.GetChBody())


motor = chrono.ChLinkMotorRotationAngle()
motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
motor.SetAngleFunction(chrono.ChFunctionRamp(0, math.pi / 4))
motor.Initialize(beam.GetChBody(), chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0)))
system.Add(motor)


flywheel = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
flywheel.SetPos(chrono.ChVector3d(0, 0, 0))
flywheel.SetFixed(True)
flywheel.Initialize()
system.Add(flywheel)


link = chrono.ChLinkLockRevolute()
link.Initialize(beam.GetChBody(), flywheel, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))))
system.Add(link)


vis = fea.ChVisualSystemFEM()
vis.SetChTime(0.1)
vis.SetSymbolscale(1.0)
vis.SetShapeColor(chrono.ChColor(0, 0, 0))
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddBeam(beam)
vis.Render()


vis_irr = irr.ChVisualSystemIrrlicht()
vis_irr.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis_irr.SetWindowSize(1024,768)
vis_irr.SetWindowTitle('Test')
vis_irr.Initialize()
vis_irr.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_irr.AddLightDirectional()
vis_irr.AddBeam(beam)
vis_irr.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_irr.AddSkyBox()
vis_irr.AttachSystem(system)


while vis_irr.Run() :
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.001)
    vis_irr.Updating()
    vis_irr.BeginScene()
    vis_irr.Render()
    vis_irr.EndScene()