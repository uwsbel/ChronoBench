import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)  # for max precision in gravity of FE with cubic IGA
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -3.71, 0))

beam_L = 10
beam_ro = 0.060
beam_ri = 0.055
CH_PI = m.pi  # Use math.pi for precision

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetJ((CH_PI / 2.0) * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,  # mesh
                 msection,  # section
                 20,  # number of elements
                 chrono.ChVectorD(0, 0, 0),  # start
                 chrono.ChVectorD(beam_L, 0, 0),  # end
                 chrono.VECT_Y,  # section direction
                 3)  # cubic IGA (order 3)

nodes = builder.GetLastBeamNodes()
mid_index = len(nodes) // 2
node_mid = nodes[mid_index]

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0))
mbodyflywheel.SetRot(chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1], truss, chrono.ChFrameD(builder.GetLastBeamNodes()[-1].GetPos()))
sys.Add(bearing)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes()[0], truss, chrono.ChFrameD(builder.GetLastBeamNodes()[0].GetPos()))
sys.Add(rotmotor1)

f_ramp = chrono.ChFunction_Sine(60, 0.1)
rotmotor1.SetMotorFunction(f_ramp)

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, 8), chrono.ChVectorD(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)
sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)