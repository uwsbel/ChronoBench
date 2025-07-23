import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Create the system
sys = chrono.ChSystemSMC()

# Create the mesh
mesh = fea.ChMesh()
sys.Add(mesh)

# Settings for gravity and mesh
mesh.SetAutomaticGravity(True, 2)  
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  # Changed gravity

# Beam properties
beam_L = 10  # Changed beam length
beam_ro = 0.060  # Changed outer radius
beam_ri = 0.055  # Changed inner radius

# Create a section for the beam
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

# Build the beam
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(beam_L, 0, 0), chrono.VECT_Y, 1)

# Get the middle node of the beam
nodes = builder.GetLastBeamNodes()
if len(nodes) > 0:
    node_mid = nodes[m.floor(len(nodes) / 2.0)]
else:
    print("Error: No nodes found.")
    # Handle the error

# Create and add the flywheel
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # Changed radius
mbodyflywheel.SetCoordsys(chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0), 
                                              chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Z)))
sys.Add(mbodyflywheel)

# Fix the flywheel to the beam
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss and fix it
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(nodes[-1], truss, chrono.ChFramed(nodes[-1].GetPos()))
sys.Add(bearing)

# Create the motor
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(nodes[0], truss, chrono.ChFramed(nodes[0].GetPos(), 
                                                      chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

# Set the motor function
f_ramp = chrono.ChFunctionSine(60, 0.1)  # Changed motor function
rotmotor1.SetMotorFunction(f_ramp)

# Visualize the FEM mesh
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

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # Changed camera position
vis.AddTypicalLights()

# Set the solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Perform static analysis
sys.DoStaticLinear()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)