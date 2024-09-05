import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.ChSystemSMC()
system = chrono.ChSystem()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Epicyclic Gears Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()




truss_body = chrono.ChBodyEasyBox(1, 0.1, 0.1, 1000, True, True)
truss_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(truss_body)


bar_body = chrono.ChBodyEasyCylinder(0.05, 0.5, 1000, True, True)
bar_body.SetPos(chrono.ChVectorD(0, 0.25, 0))
bar_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(bar_body)


gear1_body = chrono.ChBodyEasyGear(0.1, 0.05, 10, 1000, True, True)
gear1_body.SetPos(chrono.ChVectorD(0, 0.25, 0))
gear1_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(gear1_body)


gear2_body = chrono.ChBodyEasyGear(0.05, 0.025, 10, 1000, True, True)
gear2_body.SetPos(chrono.ChVectorD(0.15, 0.25, 0))
gear2_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(gear2_body)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1_body, truss_body, chrono.ChFrameD(chrono.ChVectorD(0, 0.25, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(10))
system.Add(motor)




contact_method = chrono.ChMaterialSurface.SMC
gear1_body.GetMaterialSurface()->SetFriction(0.3)
gear2_body.GetMaterialSurface()->SetFriction(0.3)
chrono.ChCollisionModel::SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel::SetDefaultSuggestedMargin(0.001)
gear1_body.GetCollisionModel()->ClearModel()
gear1_body.GetCollisionModel()->AddCylinder(contact_method, 0.05, 0.025, chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
gear1_body.GetCollisionModel()->BuildModel()
gear2_body.GetCollisionModel()->ClearModel()
gear2_body.GetCollisionModel()->AddCylinder(contact_method, 0.025, 0.0125, chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
gear2_body.GetCollisionModel()->BuildModel()


bar_body.GetCollisionModel()->ClearModel()
bar_body.GetCollisionModel()->AddCylinder(contact_method, 0.025, 0.25, chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))
bar_body.GetCollisionModel()->BuildModel()


while vis.Run():
    system.DoStepDynamics(0.01)
    vis.Render()