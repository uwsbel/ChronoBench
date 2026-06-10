import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

try:
    import pychrono.pardisomkl as mkl
except ImportError:
    mkl = None


sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))

# Modified beam dimensions
beam_L = 10
beam_ro = 0.060
beam_ri = 0.055
CH_PI = m.pi

VECT_Y = chrono.ChVector3d(0, 1, 0)
VECT_Z = chrono.ChVector3d(0, 0, 1)

# Create a section, i.e. thickness and material properties for beams.
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

# Use the ChBuilderBeamIGA tool for creating a straight rod.
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    20,
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    VECT_Y,
    1
)

# Safer Python-style access to generated beam nodes.
beam_nodes = builder.GetLastBeamNodes()
num_nodes = int(beam_nodes.size()) if hasattr(beam_nodes, "size") else len(beam_nodes)

node_start = beam_nodes[0]
node_mid = beam_nodes[num_nodes // 2]
node_end = beam_nodes[num_nodes - 1]

# Create the flywheel and attach it to the center of the beam.
mbodyflywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y,
    0.30,
    0.1,
    7800
)

mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, VECT_Z)
    )
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss.
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create the end bearing.
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    node_end,
    truss,
    chrono.ChFramed(node_end.GetPos())
)
sys.Add(bearing)

# Create the motor that rotates the beam.
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    node_start,
    truss,
    chrono.ChFramed(
        node_start.GetPos(),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, VECT_Y)
    )
)
sys.Add(rotmotor1)

# Modified motor function.
f_ramp = chrono.ChFunctionSine(60, 0.1)

# Support both older and newer PyChrono method names.
if hasattr(rotmotor1, "SetMotorFunction"):
    rotmotor1.SetMotorFunction(f_ramp)
else:
    rotmotor1.SetSpeedFunction(f_ramp)

# Attach visualization of the FEM mesh.
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

# Create the Irrlicht visualization.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Test FEA: the Jeffcott rotor with IGA beams")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 2, 8),
    chrono.ChVector3d(beam_L / 2, 0, 0)
)
vis.AddTypicalLights()

# Solver setup.
if mkl is not None:
    msolver = mkl.ChSolverPardisoMKL()
else:
    msolver = chrono.ChSolverMINRES()
    msolver.SetMaxIterations(200)
    msolver.SetTolerance(1e-10)
    msolver.EnableDiagonalPreconditioner(True)

sys.SetSolver(msolver)

# Static preload.
sys.DoStaticLinear()

# Simulation loop.
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)