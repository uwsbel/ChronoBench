import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using Euler-Bernoulli beam finite elements")

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh for finite elements
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)

# Create a beam section for Euler-Bernoulli beam
section = fea.ChBeamSectionEulerAdvanced()
section.SetAsRectangularSection(0.012, 0.025)
section.SetYoungModulus(0.01e9)
section.SetShearModulus(0.01e9 * 0.3)
section.SetRayleighDamping(0.000)
section.SetCentroid(0, 0.02)
section.SetShearCenter(0, 0.1)
section.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)

# Use ChBuilderBeamEuler to create a beam from (0,0,-0.1) to (0.2,0,-0.1)
builder = chrono.ChBuilderBeamEuler()
builder.BuildBeam(sys, mesh, section, chrono.ChVector3d(0, 0, -0.1), chrono.ChVector3d(0.2, 0, -0.1), chrono.VECT_Y, 5)

# Apply a force to the first node of the beam
first_node = builder.GetLastBeamNodes()[0]
first_node.SetForce(chrono.ChVector3d(0, -1, 0))

# Fix the last node of the beam using a constraint
last_node = builder.GetLastBeamNodes()[1]
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

constr_last = chrono.ChLinkMateGeneric()
constr_last.Initialize(last_node, truss, False, last_node.Frame(), last_node.Frame())
sys.Add(constr_last)
constr_last.SetConstrainedCoords(True, True, True, True, True, True)

# Add visualization for the beam and nodes
visualizebeam = chrono.ChVisualShapeFEA(mesh)
visualizebeam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeam.SetColorscaleMinMax(-0.4, 0.4)
visualizebeam.SetSmoothFaces(True)
visualizebeam.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeam)

visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

sys.Add(mesh)

# Set up the MKL Pardiso solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Create visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA Euler-Bernoulli beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)