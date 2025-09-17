import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)  # for max precision in gravity of FE
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Modified gravity

beam_L = 10  # Modified beam length
beam_ro = 0.060  # Modified outer radius
beam_ri = 0.055  # Modified inner radius

# Create section with proper CH_PI
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(chrono.CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((chrono.CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build beam with IGA
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, 
                  chrono.ChVector3d(0, 0, 0), 
                  chrono.ChVector3d(beam_L, 0, 0), 
                  chrono.VECT_Y, 1)

# Get middle node using integer division
nodes = builder.GetLastBeamNodes()
node_mid = nodes[len(nodes) // 2]

# Create flywheel with modified radius
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # Modified radius
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(chrono.CH_PI / 2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)

# Fix flywheel to beam
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create bearing with corrected constraints
bearing = chrono.ChLinkMateGeneric(True, True, True,  # Constrain translations
                                   False, True, True)  # Allow rotation about X only
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Create motor with corrected frame alignment
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),
                     truss,
                     chrono.ChFrameD(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(-chrono.CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

# Set modified motor function
f_ramp = chrono.ChFunctionSine(60, 0.1)  # Modified motor function
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

# Irrlicht setup with modified camera position
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Jeffcott Rotor with IGA Beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # Modified camera position
vis.AddTypicalLights()

# Solver and simulation
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)