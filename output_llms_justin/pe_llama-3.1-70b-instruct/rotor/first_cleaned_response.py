import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


sys = chrono.ChSystemSMC()


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data'))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


iga_beam = fea.ChBeamIGA()
iga_beam.SetYoungModulus(2e11)
iga_beam.SetShearModulus(8e10)
iga_beam.SetDensity(7800)
iga_beam.SetLength(1.0)
iga_beam.SetNumCtrlPts(10)
iga_beam.SetNumElems(10)
iga_beam.SetCrossSectDiameter(0.05)


flywheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 7800, True, True)
flywheel.SetPos(chrono.ChVector3d(0.5, 0, 0))
flywheel.SetRot(chrono.Q_from_AngX(math.pi / 2))


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(iga_beam, flywheel, chrono.ChCoordsysD(chrono.ChVector3d(0.5, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(10.0))


sys.Add(iga_beam)
sys.Add(flywheel)
sys.Add(motor)


vis.AttachSystem(sys)
vis.AddCameraCameraMotion(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), 0.1, 0.1)
vis.AddLightWithShadow(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), 10, 10, 10, 10, 100)
vis.AddFEMmeshVisualization(iga_beam, fea.ChVisualizationFEAMesh.MESH_WIREFRAME)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()