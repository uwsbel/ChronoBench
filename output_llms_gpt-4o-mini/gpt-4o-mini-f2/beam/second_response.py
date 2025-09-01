import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print ("Example: PyChrono using beam finite elements");

sys = chrono.ChSystemSMC()

# Create a mesh, that is a container for groups of
# elements and their referenced nodes.
mesh = fea.ChMesh();

# Create a section, i.e. thickness and material properties,
# for beams. This will be shared among some beams.
msection = fea.ChBeamSectionEulerAdvanced()

# Set the width and height of the rectangular section of the beam.
beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)

# Set the material properties of the beam.
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
msection.SetCentroid(0,0.02)
msection.SetShearCenter(0,0.1)
msection.SetSectionRotation(45*chrono.CH_RAD_TO_DEG)

# Now, we create some beams, that are sequences of
# ChNodeFEAxyzrot nodes, connected by ChElementBeamEuler elements.

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

# Fix node 1 to ground (truss)
# hnode1.SetFixed(True)
mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,   # x, y, z
                               True, True, True)   # Rx, Ry, Rz

constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,    # x, y, z
                              False, False,False)   # Rx, Ry, Rz


# Create a section of Euler-Bernoulli beams
# along two ChVector3d points.

# This is the helper object that will create
# a set of FEM elements and nodes for you.
builder = fea.ChBuilderBeamEuler()

# These are the two points that define the beam's line.
pointA = chrono.ChVector3d(0, 0, -0.1)
pointB = chrono.ChVector3d(0.2, 0, -0.1)

# Now we can use the builder to create a beam
# from point A to point B:
builder.BuildBeam(mesh,                   # the mesh where to put the created nodes and elements
                  msection,               # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                  5,                     # number of ChElementBeamEuler to create
                  pointA, pointB,        # the two ChVector3d points
                  chrono.ChVector3d(0, 1, 0) # the 'Y' up direction for the section
                  );

# Fix the last node of the created beam
builder.GetLastBeamNodes().back().SetFixed(True)

# Apply a force to the first node of the created beam
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))


# We do not want gravity effect on FEA elements in this demo.
mesh.SetAutomaticGravity(False);

# Remember to add the mesh to the system!
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

# Create an Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)


# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)