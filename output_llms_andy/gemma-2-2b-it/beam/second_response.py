import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using Euler-Bernoulli beams") 

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# Create a section object for beam properties. This will define the characteristics of all beams that use this section.
msection = fea.ChBuilderBeamEuler(
    chrono.ChVector3d(0, 0, -0.1),  # Start point of the beam
    chrono.ChVector3d(0.2, 0, -0.1),  # End point of the beam
    chrono.ChVector3d(0, 1, 0),  # Y direction
    5  # Number of elements
)

# Set the material properties of the beam.
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0, 0.02)
msection.SetShearCenter(0, 0.1)
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)

# Create the first beam element and set its properties.
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(msection.GetNodes()[0], msection.GetNodes()[1])  # Connect the first beam between node 1 and node 2.
belement1.SetSection(msection) 
mesh.AddElement(belement1)

# Create the second beam element and set its properties.
belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(msection.GetNodes()[1], msection.GetNodes()[2])  # Connect the second beam between node 2 and node 3.
belement2.SetSection(msection) 
mesh.AddElement(belement2)

# Apply a force to node 1: (0, -1, 0)
msection.GetNodes()[0].SetForce(chrono.ChVector3d(0, -1, 0)) 

# Create a fixed truss, which is a rigid body that won't move.
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Create and initialize a constraint that connects node 1 to the fixed truss.
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(msection.GetNodes()[0], mtruss, False, msection.GetNodes()[0].Frame(), msection.GetNodes()[0].Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True)  # Constrain x, y, z translations.
constr_bc.SetConstrainedCoords(True, True, True)  # Constrain Rx, Ry, Rz rotations.

# Create and initialize a constraint that connects node 2 to the fixed truss.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(msection.GetNodes()[1], mtruss, False, msection.GetNodes()[1].Frame(), msection.GetNodes()[1].Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True)  # Constrain only y, z translations.
constr_d.SetConstrainedCoords(False, False, False)  # Do not constrain any rotations.

# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)

# Add the mesh to the physical system.
sys.Add(mesh)

# Add visualization for the beams in the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for the nodes in the mesh.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create an Irrlicht visualization window.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Change the default solver to the MKL Pardiso solver, which is more precise for FEA.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop.
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)