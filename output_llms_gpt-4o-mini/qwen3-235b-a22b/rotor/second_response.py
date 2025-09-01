import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

# Corrected beam parameters per instructions
beam_L = 10
beam_ro = 0.060
beam_ri = 0.055
CH_PI = 3.141592653589793  # Corrected value of π

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Changed gravity

# Create beam section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build beam
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(beam_L, 0, 0), chrono.VECT_Y, 1)

# Get middle node (corrected from .size() to len())
node_mid = builder.GetLastBeamNodes()[int(len(builder.GetLastBeamNodes()) / 2)]

# Create flywheel with updated radius
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z))
)
sys.Add(mbodyflywheel)

# Fix joint for flywheel
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# End bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(), truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Add motor body to connect to beam node
motor_node_body = chrono.ChBody()
motor_node_body.SetMass(0.001)
motor_node_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
sys.Add(motor_node_body)

# Fix motor body to front node
motor_fix = chrono.ChLinkMateFix()
motor_fix.Initialize(builder.GetLastBeamNodes().front(), motor_node_body)
sys.Add(motor_fix)

# Create motor with updated sine function
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(motor_node_body, truss,
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(60, 0.1)  # Updated motor function
rotmotor1.SetMotorFunction(f_ramp)

# Visualization
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

# Irrlicht visualization with updated camera
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # Updated camera
vis.AddTypicalLights()

# Solver setup
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Remove static analysis call for dynamic simulation
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)