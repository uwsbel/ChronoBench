import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Initialize the system with Smooth Contact (SMC) method
sys = chrono.ChSystemSMC()

# Create and add a finite element mesh
mesh = fea.ChMesh()
sys.Add(mesh)

# Enable automatic gravity for the mesh with 2 integration points per element
mesh.SetAutomaticGravity(True, 2)
# Set gravitational acceleration to simulate a lower-gravity environment
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))

# Define beam geometry
beam_L = 10  # Updated beam length
beam_ro = 0.060  # Updated outer radius
beam_ri = 0.055  # Updated inner radius
CH_PI = 3.1456  # Approximation of π used in the simulation

# Create beam section properties
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

# Create beam section
msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build the beam using IGA
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    20,  # Number of beam elements
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(beam_L, 0, 0),
    chrono.VECT_Y,
    1  # Order of the beam (1 = linear)
)

# Get the middle node of the beam
nodes = builder.GetLastBeamNodes()
node_mid = nodes[int(m.floor(len(nodes) / 2.0))]

# Create and position the flywheel
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # Updated radius
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z)
    )
)
sys.Add(mbodyflywheel)

# Attach the flywheel to the beam using a fixed joint
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss (fixed body)
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create the bearing at the beam's end
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    builder.GetLastBeamNodes()[-1],
    truss,
    chrono.ChFramed(builder.GetLastBeamNodes()[-1].GetPos())
)
sys.Add(bearing)

# Create and configure the motor
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes()[0],
    truss,
    chrono.ChFramed(
        builder.GetLastBeamNodes()[0].GetPos(),
        chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)
    )
)
sys.Add(rotmotor1)

# Set the motor function to a sine wave with updated parameters
f_ramp = chrono.ChFunctionSine(60, 0.1)  # Updated frequency and amplitude
rotmotor1.SetMotorFunction(f_ramp)

# Add FEA visualization
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

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # Updated camera position
vis.AddTypicalLights()

# Set the solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)