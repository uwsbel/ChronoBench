import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("/path/to/chrono/data/")


sys = chrono.ChSystemSMC()


beam = fea.ChElementBeamIGA()
beam.SetNodes(fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(0))),
              fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(1, 0, 0), chrono.Q_from_AngZ(0))))


flywheel = chrono.ChBodyEasyCylinder(chrono.ChVectorD(0, 0, 0), 0.2, 0.1, 1000, True, True)
flywheel.SetBodyFixed(False)
sys.Add(flywheel)


mesh = fea.ChMesh()
mesh.AddElement(beam)
sys.Add(mesh)


beam_node = beam.GetNodeN(0)
constrain = chrono.ChLinkMateGeneric()
constrain.Initialize(beam_node, flywheel, False, beam_node.GetFrame_REF_to_abs(), flywheel.GetFrame_REF_to_abs())
sys.Add(constrain)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetNodeN(0), chrono.ChFrame(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(0)), chrono.ChFrame(chrono.ChVectorD(-1, 0, 0), chrono.Q_from_AngZ(0)))
motor.SetSpeedFunction(chrono.ChFunction_Ramp(0, 1))
sys.Add(motor)


vis_mesh = fea.ChVisualShapeFEA(mesh)
vis_mesh.SetFEMdataType(fea.ChVisualShapeFEA.E_PLOT_ELEM_BEAM_MZ)
vis_mesh.SetColorscaleMinMax(-0.5, 0.5)
vis_mesh.SetSmoothFaces(True)
vis_mesh.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('IGA Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, -3))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)