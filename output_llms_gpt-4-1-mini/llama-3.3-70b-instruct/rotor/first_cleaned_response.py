import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


sys = chrono.ChSystemNSC()


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  
sys.SetStep(1e-3)  


beam = fea.ChBeamSectionEuler()
beam.SetE(2.1e11)  
beam.SetIy(1e-5)  
beam.SetIz(1e-5)  
beam.SetA(1e-4)  
beam.SetAs(1e-4)  
beam.SetJ(1e-6)  
beam.SetG(8e10)  
beam.SetRho(7850)  


beam_body = fea.ChBodyEasyBeam(beam, 10, 1.0, 0.1, 0.1, 0.1)
beam_body.SetPos(chrono.ChVectorD(0, 0, 0))  
beam_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
sys.Add(beam_body)


flywheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 7850)
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))  
flywheel.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  
sys.Add(flywheel)


motor = chrono.ChLinkMotorRotation()
motor.Initialize(beam_body, sys.GetGroundBody(), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetMotorFunction(chrono.ChFunctionRotationSpeed(chrono.ChVectorD(0, 0, 1), 10))
sys.Add(motor)


material = chrono.ChMaterialSurfaceNSC()
material.SetFriction(0.5)
material.SetDampingF(0.5)
material.SetCompliance(1e-6)
material.SetComplianceT(1e-6)


contact = chrono.ChLinkMateGeneric()
contact.Initialize(beam_body, flywheel, chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
contact.SetMaterial(material)
sys.Add(contact)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 3))
vis.AddCamera(chrono.ChVectorD(0, 3, 0))
vis.AddCamera(chrono.ChVectorD(3, 0, 0))


fem_vis = fea.ChVisualizationFEAbeam()
fem_vis.AddColorBar()
fem_vis.AddBeam(beam_body)


while vis.Run():
    sys.DoStepDynamics(1e-3)
    vis.BeginScene()
    vis.DrawAll()
    fem_vis.Draw()
    vis.EndScene()