import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


out_dir = chrono.GetChronoOutputPath() + "JEFFCOTT_ROTOR_IGA"

print( "Copyright (c) 2023 Project Chrono")


sys = chrono.ChSystemNSC()


mesh = fea.ChMesh()


material = fea.ChMaterialBeamIGA()
material.SetYoungModulus(210e9)
material.SetPoissonRatio(0.3)
material.SetDensity(7800)


builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, material, 10, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0), 12, 0.010)


node_mid = builder.GetLastBeamNodes()[int(len(builder.GetLastBeamNodes()) / 2)]
node_tip = builder.GetLastBeamNodes()[-1]


body_flywheel = chrono.ChBodyEasyCylinder(chrono.ChVector3d(0, 0, 0), 0.2, 0.050, 1000, True, True)
body_flywheel.SetPos(node_mid.GetPos())
body_flywheel.SetMass(10)
body_flywheel.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
sys.Add(body_flywheel)


constraint = chrono.ChLinkMateFix()
constraint.Initialize(node_mid, body_flywheel)
sys.Add(constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(builder.GetLastBeamNodes()[0], chrono.ChFrame(chrono.ChVector3d(-0.2, 0, 0), chrono.Q_from_AngAxis(chrono.CH_PI / 2, chrono.VECT_Z)))
sys.Add(motor)


motor_fun = chrono.ChFunction_Const(chrono.CH_PI)
motor.SetSpeedFunction(motor_fun)


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Jeffcott Rotor IGA Beam Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2, 0.2, 0.4))
vis.AddTypicalLights()


fem_vis = fea.ChVisualShapeFEA(mesh)
fem_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
fem_vis.SetColorscaleMinMax(-0.4, 0.4)
fem_vis.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(fem_vis)

fem_vis = fea.ChVisualShapeFEA(mesh)
fem_vis.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_CSYS)
fem_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
fem_vis.SetSymbolsThickness(0.006)
fem_vis.SetSymbolsScale(0.01)
fem_vis.SetZbufferHide(False)
mesh.AddVisualShapeFEA(fem_vis)


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)