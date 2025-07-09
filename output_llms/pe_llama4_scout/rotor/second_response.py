import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Create the simulation system
sys = chrono.ChSystemSMC()

# Add the FEA mesh to the system
mesh = fea.ChMesh()
sys.Add(mesh)

# Set automatic gravity for the FEA mesh
mesh.SetAutomaticGravity(True, 2)

# Set gravitational acceleration
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))

# Beam properties
beam_L = 10
beam_ro = 0.060
beam_ri = 0.055

# Create a section for beams
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(math.pi * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((math.pi / 4.0) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((math.pi / 4.0) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((math.pi / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetIzz((math.pi / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetJ((math.pi / 2.0) * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Create the beam using ChBuilderBeamIGA
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(beam_L, 0, 0), chrono.VECT_Y, 1)

node_mid = builder.GetLastBeamNodes()[int(len(builder.GetLastBeamNodes()) / 2)]

# Create the flywheel
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(math.pi / 2.0, chrono.VECT_Z))
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos())
                   )
sys.Add(bearing)

# Create the motor
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),
                     truss,
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(math.pi / 2.0, chrono.VECT_Y))
                     )
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(60, 0.1)
rotmotor1.SetMotorFunction(f_ramp)

# Add visualization for the FEM mesh
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

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)