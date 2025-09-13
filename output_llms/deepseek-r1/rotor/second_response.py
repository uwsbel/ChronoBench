import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)  # Max precision for gravity in FE
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Changed to lower gravity

beam_L = 10  # Changed from 6 to 10
beam_ro = 0.060  # Changed from 0.050
beam_ri = 0.055  # Changed from 0.045

# Use math.pi instead of approximate value
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(m.pi * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((m.pi / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build beam with IGA
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                  msection,
                  20,  # number of sections
                  chrono.ChVector3d(0, 0, 0),
                  chrono.ChVector3d(beam_L, 0, 0),
                  chrono.VECT_Y,  # section direction
                  1)  # linear elements

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]

# Create flywheel with new radius
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # Radius changed to 0.30
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)

# Fix flywheel to beam center
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Corrected bearing: Fixed position, free rotation about X
bearing = chrono.ChLinkMateGeneric(True, True, True, False, True, True)  # Fixed translation
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Motor with new parameters
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),
                     truss,
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

f_ramp = chrono.ChFunctionSine(60, 0.1)  # Changed to (60, 0.1)
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

# Irrlicht setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Jeffcott Rotor with IGA Beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # New camera position
vis.AddTypicalLights()

# Solver setup
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop
sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)