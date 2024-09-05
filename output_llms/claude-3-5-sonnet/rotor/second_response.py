import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -3.71, 0))  # Changed gravity

beam_L = 10  # Changed beam length
beam_ro = 0.060  # Changed outer radius
beam_ri = 0.055  # Changed inner radius
CH_PI = m.pi  # Corrected PI value

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetGShearModulus(210e9 / (2 * (1 + 0.3)))  # Corrected shear modulus calculation
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)

msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                  msection,
                  20,
                  chrono.ChVectorD(0, 0, 0),
                  chrono.ChVectorD(beam_L, 0, 0),
                  chrono.VECT_Y,
                  1)

node_mid = builder.GetLastBeamNodes()[m.floor(len(builder.GetLastBeamNodes()) / 2.0)]

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # Changed flywheel radius
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0))
mbodyflywheel.SetRot(chrono.Q_ROTATE_X_TO_Y)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1],
                   truss,
                   chrono.ChFrameD(builder.GetLastBeamNodes()[-1].GetPos())
                   )
sys.Add(bearing)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()

rotmotor1.Initialize(builder.GetLastBeamNodes()[0],
                     truss,
                     chrono.ChFrameD(builder.GetLastBeamNodes()[0].GetPos(),
                                     chrono.Q_ROTATE_Y_TO_Z)
                     )
sys.Add(rotmotor1)

f_ramp = chrono.ChFunction_Sine(60, 0.1)  # Changed motor function
rotmotor1.SetSpeedFunction(f_ramp)

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
vis.AddCamera(chrono.ChVectorD(0, 2, 8), chrono.ChVectorD(beam_L / 2, 0, 0))  # Changed camera position
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)