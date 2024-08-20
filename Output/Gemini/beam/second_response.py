import pychrono as chrono  # Import the main PyChrono module.
import pychrono.fea as fea  # Import the finite element analysis (FEA) module.
import pychrono.pardisomkl as mkl  # Import the Pardiso MKL linear solver module.
import pychrono.irrlicht as chronoirr  # Import the Irrlicht visualization module.

print("Example: PyChrono using beam finite elements")  # Print an introductory statement.

# Create the physical system that will be simulated.
sys = chrono.ChSystemSMC()

# ----------------------------------------------------------------------------------------------
# Add a Section on Euler-Bernoulli Beams using ChBuilderBeamEuler
# ----------------------------------------------------------------------------------------------

# Create a mesh for the beam.
mesh = fea.ChMesh()

# Create a section object for beam properties.
msection = fea.ChBeamSectionEulerAdvanced()

# Set the width and height of the rectangular section of the beam.
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)  # Set the cross-sectional shape and size.

# Set the material properties of the beam.
msection.SetYoungModulus(0.01e9)  # Young's modulus
msection.SetShearModulus(0.01e9 * 0.3)  # Shear modulus
msection.SetRayleighDamping(0.000)  # Damping coefficient
# msection.SetCentroid(0, 0.02) # Not necessary, automatically handled
# msection.SetShearCenter(0, 0.1) # Not necessary, automatically handled
# msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)  # No need to set rotation

# Create a ChBuilderBeamEuler object to help create the beam.
builder = fea.ChBuilderBeamEuler(mesh)

# Define the beam section geometry
builder.SetBeamSection(msection)  # Set the beam section properties.
builder.SetIntegrationPoints(2)  # Use 2 integration points per beam element.

# Point A and B for the beam.
pointA = chrono.ChVectorD(0, 0, -0.1)
pointB = chrono.ChVectorD(0.2, 0, -0.1)

# Build the beam using the builder object.
builder.BuildBeam(pointA,  # Starting point of the beam.
                  pointB,  # Ending point of the beam.
                  chrono.VECT_Y,  # 'Y' up direction.
                  5)  # Number of elements in the beam.

# Fix the last node of the created beam.
builder.GetLastBeamNodes().back().SetFixed(True)

# Apply a force to the first node of the beam.
builder.GetLastBeamNodes()[0].SetForce(chrono.ChVectorD(0, -1, 0))

# Add the mesh to the system.
sys.Add(mesh)

# ----------------------------------------------------------------------------------------------
# Previous code with modifications
# ----------------------------------------------------------------------------------------------

# Define the length of the beam elements.
beam_L = 0.1

# Create nodes for the positions that will be used for beams.
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))  # Node at origin.
hnode2 = fea.ChNodeFEAxyzrot(
    chrono.ChFrameD(chrono.ChVectorD(beam_L, 0, 0))
)  # Node at beam length.
hnode3 = fea.ChNodeFEAxyzrot(
    chrono.ChFrameD(chrono.ChVectorD(beam_L * 2, 0, 0))
)  # Node at twice the beam length.

# Add nodes to the mesh.
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# Create the first beam element and set its properties.
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)  # Connect the first beam between node 1 and node 2.
belement1.SetSection(msection)  # Assign the section properties to this beam element.
mesh.AddElement(belement1)  # Add the first beam element to the mesh.

# Create the second beam element and set its properties.
belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)  # Connect the second beam between node 2 and node 3.
belement2.SetSection(msection)  # Assign the section properties to this beam element.
mesh.AddElement(belement2)  # Add the second beam element to the mesh.

# Apply a force to node 2: 4 N in the x-direction and 2 N in the y-direction.
hnode2.SetForce(chrono.ChVectorD(4, 2, 0))

# Apply a torque to node 3: -0.04 Nm in the y-direction.
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))

# Create a fixed truss, which is a rigid body that won't move.
mtruss = chrono.ChBody()
mtruss.SetBodyFixed(True)  # Fix the truss so it won't move.
sys.Add(mtruss)  # Add the truss to the physical system.

# Modify Existing Node-Fixing Approach: Use ChLinkMateGeneric
# ----------------------------------------------------------------------------------------------

# Create and initialize a constraint that connects node 3 to the fixed truss.
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)  # Add the constraint to the system.
constr_bc.SetConstrainedCoords(
    True, True, True, True, True, True
)  # Constrain all degrees of freedom

# Create and initialize a constraint that connects node 1 to the fixed truss.
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)  # Add the constraint to the system.
constr_d.SetConstrainedCoords(
    False, True, True, False, False, False
)  # Constrain only y, z translations

# ----------------------------------------------------------------------------------------------

# Disable the automatic gravity for FEA elements in this demonstration.
mesh.SetAutomaticGravity(False)

# Visualization for the beams in the mesh (unchanged).
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Visualization for the nodes in the mesh (unchanged).
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create an Irrlicht visualization window.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the simulation system to the visual system.
vis.SetWindowSize(1024, 768)  # Set the window size.
vis.SetWindowTitle("FEA beams")  # Set the window title.
vis.Initialize()  # Initialize the visual system.
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # Add the Chrono logo.
vis.AddSkyBox()  # Add a skybox.
vis.AddCamera(chrono.ChVectorD(0.1, 0.1, 0.2))  # Add a camera.
vis.AddTypicalLights()  # Add typical lights for the scene.

# Set up the MKL Pardiso solver as the system solver (unchanged).
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Simulation loop.
while vis.Run():
    vis.BeginScene()  # Begin the scene.
    vis.Render()  # Render the scene.
    vis.EndScene()  # End the scene.
    sys.DoStepDynamics(0.001)  # Perform one step of simulation