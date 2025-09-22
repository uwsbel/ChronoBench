import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Corrected: Use print() as a function for Python 3 compatibility
print("Example: PyChrono using beam finite elements")

# --- System Creation ---
sys = chrono.ChSystemSMC()

# --- Mesh Creation ---
# Create a mesh, that is a container for groups of elements and their referenced nodes.
mesh = fea.ChMesh()

# --- Beam Section Definition ---
# Create a section, i.e. thickness and material properties for beams.
# This will be shared among some beams.
msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012  # Beam width in local y direction
beam_wz = 0.025  # Beam height in local z direction
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3) # G = E * 0.3, implies a specific Poisson's ratio if isotropic
msection.SetRayleighDamping(0.000)

# Advanced section properties:
msection.SetCentroid(0, 0.02)  # Offset of elastic centroid
msection.SetShearCenter(0, 0.1) # Offset of shear center
# Corrected: Angle for SetSectionRotation should be in radians.
# If 45 degrees is intended, convert using CH_DEG_TO_RAD.
msection.SetSectionRotation(45 * chrono.CH_DEG_TO_RAD) # Rotate section by 45 degrees

# --- Manually Created Beams ---
# Add some EULER-BERNOULLI BEAMS:
beam_L = 0.1

# Nodes for manually created beams
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))     # (0.1, 0, 0)
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0))) # (0.2, 0, 0)

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

# First beam element (manual)
belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

# Second beam element (manual)
belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2) # Semicolon removed for Pythonic style

# --- Loads and Boundary Conditions for Manually Created Beams ---
# Apply a force or a torque to a node:
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# Fix a node to ground:
#    hnode1.SetFixed(True)  # This was commented out in original
# otherwise fix it using constraints:

# Ground body for constraints
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

# Fix hnode3 to ground (all 6 DOFs)
constr_bc = chrono.ChLinkMateGeneric()
# Using GetAbsFrame() as it returns ChFrameD, while Frame() returns ChFrameMovingD (though compatible)
constr_bc.Initialize(hnode3, mtruss, False, hnode3.GetAbsFrame(), hnode3.GetAbsFrame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,   # x, y, z
                               True, True, True)   # Rx, Ry, Rz

# Partially fix hnode1 to ground (y, z translations fixed)
constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.GetAbsFrame(), hnode1.GetAbsFrame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,     # x, y, z (x free)
                              False, False, False)    # Rx, Ry, Rz (all free)


# --- Beams Created with ChBuilderBeamEuler (the fast way!) ---
# Shortcut!
# This ChBuilderBeamEuler helper object is very useful because it will
# subdivide 'beams' into sequences of finite elements of beam type.
builder = fea.ChBuilderBeamEuler()

# First beam created by the builder:
# Spans from (0, 0, -0.1) to (0.2, 0, -0.1)
num_elements_beam1 = 5
beam1_ptA = chrono.ChVector3d(0, 0, -0.1)
beam1_ptB = chrono.ChVector3d(0.2, 0, -0.1)
beam1_Y_dir = chrono.ChVector3d(0, 1, 0)

builder.BuildBeam(mesh,
                  msection,
                  num_elements_beam1,
                  beam1_ptA,
                  beam1_ptB,
                  beam1_Y_dir)

# Retrieve nodes of the first builder beam to apply specific boundary conditions.
# builder.GetLastBeamNodes() returns a ChNodeFEAxyzrotList for the most recently built beam.
first_builder_beam_nodes = builder.GetLastBeamNodes()
node_A_first_builder = first_builder_beam_nodes.front() # Start node at beam1_ptA
node_B_first_builder = first_builder_beam_nodes.back()  # End node at beam1_ptB

# Apply boundary conditions to this first builder beam as per original script's intent:
# Fix its 'B' end (last node).
node_B_first_builder.SetFixed(True) # Node at (0.2, 0, -0.1) is fixed
# Apply a force to its 'A' end (first node).
node_A_first_builder.SetForce(chrono.ChVector3d(0, -1, 0)) # Force at (0, 0, -0.1)


# --- MODIFICATION: Add another beam segment as per instructions ---
# This new beam segment starts from node_B_first_builder (the end node of the first builder beam).
# End point for the new beam: (0.2, 0.1, -0.1)
# Y up direction: (0, 1, 0)

start_node_for_new_beam = node_B_first_builder # This is the ChNodeFEAxyzrot object at (0.2, 0, -0.1)
new_beam_endpoint_coords = chrono.ChVector3d(0.2, 0.1, -0.1)
new_beam_Y_up_direction = chrono.ChVector3d(0, 1, 0)
num_elements_for_new_beam = 5 # Number of elements for the new beam segment

builder.BuildBeam(mesh,
                  msection,
                  num_elements_for_new_beam,
                  start_node_for_new_beam,    # Starting ChNodeFEAxyzrot object
                  new_beam_endpoint_coords,   # Ending ChVector3d point coordinates
                  new_beam_Y_up_direction)
# Note: After this call, builder.GetLastBeamNodes() would refer to the nodes of this new (second) beam.
# --- END OF MODIFICATION ---


# --- FEA Settings ---
# We do not want gravity effect on FEA elements in this demo
mesh.SetAutomaticGravity(False) # Semicolon removed

# Remember to add the mesh to the system!
sys.Add(mesh)

# --- Visualization ---
# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh by setting proper
# coordinates and vertex colors as in the FEM elements.

visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ) # Show bending moment Z
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4) # Min/max for color scale
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Visualization for node coordinate systems
visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS) # Show CSYS at nodes
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE) # No data mapping
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01) # Adjust scale of CSYS glyphs
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# --- Irrlicht Visualization Setup ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # Ensure CHRONO_DATA_DIR is set
vis.AddSkyBox()
# Adjusted camera target for better view of the structures
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.3), chrono.ChVector3d(0.1, 0.05, -0.05))
vis.AddTypicalLights()

# --- Solver Configuration ---
# Change the solver from the default SOR to the MKL Pardiso, more precise for FEA.
# This requires the Pardiso MKL module to be available.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# --- Simulation Loop ---
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001) # Time step for the simulation