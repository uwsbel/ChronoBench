import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


sys = chrono.ChSystemNSC()


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedRadius(0.001)


beam = fea.ChBeamSectionElastic()
beam.SetE(2e11)  
beam.SetIy(1e-5)  
beam.SetIz(1e-5)  
beam.SetJ(1e-6)  
beam.SetA(1e-4)  
beam.SetAs(1e-4)  
beam.SetKs(0.85)  
beam.SetAlpha(0.01)  


iga_beam = fea.ChBeamIGA()
iga_beam.SetSection(beam)
iga_beam.SetL(1.0)  
iga_beam.SetN(10)  
iga_beam.SetP(3)  
iga_beam.SetR(0.01)  


sys.Add(iga_beam)


flywheel = chrono.ChBodyEasyBox(sys, 0.1, 0.1, 0.1, 7800)  
flywheel.SetPos(chrono.ChVectorD(0, 0, 0))  
flywheel.SetRot(chrono.Q_from_AngX(0))  


sys.Add(flywheel)


motor = chrono.ChLinkMotorRotation()
motor.Initialize(iga_beam, flywheel)
motor.SetMotorFunction(chrono.ChFunction_Const(10))  


sys.Add(motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 2))
vis.AddTypicalLights()


fem_vis = fea.ChVisualizationFEMGL()
fem_vis.AddBeam(iga_beam)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    fem_vis.DrawAll()
    vis.EndScene()