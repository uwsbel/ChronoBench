import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.ChSystemSMC()


beam_material = fea.ChMaterialShellIsotropic(1000, 0.3, 1e8)
beam_mesh = fea.ChMesh()
beam_section = fea.ChBeamSectionAdvanced(beam_material, 0.01, 0.01, 0.01)
beam_nodeA = fea.ChNodeFEAxyzrot(chrono.ChVectorD(0, 0, 0))
beam_nodeB = fea.ChNodeFEAxyzrot(chrono.ChVectorD(1, 0, 0))
beam_element = fea.ChElementBeamIGA(beam_section, beam_nodeA, beam_nodeB)
beam_mesh.AddElement(beam_element)


flywheel_material = chrono.ChMaterialSurfaceSMC(1000, 0.3)
flywheel_shape = chrono.ChSphereShape(0.05)
flywheel_body = chrono.ChBody(chrono.ChMaterialSurfaceSMC(1000, 0.3))
flywheel_body.SetPos(chrono.ChVectorD(0.5, 0, 0))
flywheel_body.AddAsset(flywheel_shape)
flywheel_body.SetMass(0.1)
flywheel_body.SetInertiaXX(chrono.ChVectorD(0.001, 0.001, 0.001))


motor_body = chrono.ChBody(chrono.ChMaterialSurfaceSMC(1000, 0.3))
motor_body.SetPos(chrono.ChVectorD(0, 0, 0))
motor_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
motor_body.SetMass(0)
motor_body.SetInertiaXX(chrono.ChVectorD(0, 0, 0))
motor_motor = chrono.ChLinkMotorRotationSpeed()
motor_motor.Initialize(motor_body, beam_nodeA, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor_motor.SetSpeedFunction(chrono.ChFunction_Const(10))


chrono.ChSystemSMC().Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


chrono.ChSystemSMC().Add(beam_mesh)
chrono.ChSystemSMC().Add(flywheel_body)
chrono.ChSystemSMC().Add(motor_body)
chrono.ChSystemSMC().Add(motor_motor)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.ChSystemSMC())
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor Simulation")
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chronoirr.ChCameraSceneNode(vis.GetSceneManager(), vis.GetVideoDriver(), chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 0, 0)))
vis.AddTypicalLights()


vis.AddFEAgeometry(beam_mesh)


while vis.Run():
    chrono.ChSystemSMC().DoStepDynamics(0.01)
    vis.Render()