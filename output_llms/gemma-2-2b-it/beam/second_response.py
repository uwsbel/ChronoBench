import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardiso as mkl
import pychrono.irrlicht as chronoirr

print("Example: PyChrono using Euler-Bernoulli beams")

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# Create a mesh, which is a container for elements and their referenced nodes.
mesh = fea.ChMesh()

# Create a section object for beam properties. This will define the characteristics of all beams that use this section.
msection = fea.ChBuilderBeamEuler(
    # Define the Euler-Bernoulli beam setup
    start_pos = chrono.ChVector3d(0, 0, -0.1),
    end_pos = chrono.ChVector3d(0.2, 0, -0.1),
    direction = chrono.ChVector3d(0, 1, 0),
    num_elements = 5,
    # Set the section properties
    YoungModulus = 0.01e9,
    ShearModulus = 0.01e9 * 0.3,
    RayleighDamping = 0.000,
    Centroid = chrono.ChVector3d(0, 0.02, 0),
    ShearCenter = chrono.ChVector3d(0, 0.1, 0),
    SectionRotation = 45 * chrono.CH_RAD_TO_DEG,
)

# Create a beam from point A to point B using the builder
beam = msection.BuildBeam()
# Fix the last node of the created beam using a constraint
beam.GetLastBeamNodes().back().SetFixed(True)
# Apply a force to the first node of the created beam
beam.GetFirstNode().SetForce(chrono.ChVector3d(0, -1, 0))

# Define the length of the beam elements.
beam_L = 0.1

# Create nodes for the positions that will be used for beams.
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))

# Add nodes to the mesh.
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Create the first beam element and set its properties.
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

# Create the second beam element and set its properties.
belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Apply a force to node 2: 4 N in the x-direction and 2 N in the y-direction.
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))

# Apply a torque to node 3: -0.04 Nm in the y-direction.
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# Create a fixed truss, which is a rigid body that won't move.
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Create and initialize a constraint that connects node 3 to the fixed truss.
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True) # Constrain x, y, z translations.
constr_bc.SetConstrainedCoords(True, True, True) # Constrain Rx, Ry, Rz rotations.

# Create and initialize a constraint that connects node 1 to the fixed truss.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True) # Constrain only y, z translations.
constr_d.SetConstrainedCoords(False, False, False) # Do not constrain any rotations.

# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)

# Add the mesh to the physical system.
sys.Add(mesh)

# Add visualization for the beams in the mesh.
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ) # Visualize the bending moments.
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4) # Set color scale limits.
visualizebeamA.SetSmoothFaces(True) # Smooth the faces for visualization.
visualizebeamA.SetWireframe(False) # Disable wireframe mode.
mesh.AddVisualShapeFEA(visualizebeamA)

# Add visualization for the nodes in the mesh.
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS) # Visualize coordinate systems at nodes.
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) # No data type visualization.
visualizebeamC.SetSymbolsThickness(0.006) # Set symbol thickness.
visualizebeamC.SetSymbolsScale(0.01) # Set symbol scale.
visualizebeamC.SetZbufferHide(False) # Do not hide symbols behind objects.
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