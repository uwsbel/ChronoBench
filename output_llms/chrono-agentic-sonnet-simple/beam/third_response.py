import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import os

print("Example: PyChrono using  beam finite elements")               # introductory banner

sys = chrono.ChSystemSMC()                                           # SMC system for FEA

mesh = fea.ChMesh()                                                  # container for FEA nodes/elements

msection = fea.ChBeamSectionEulerAdvanced()                          # Euler-Bernoulli beam section

beam_wy = 0.012                                                      # cross-section width
beam_wz = 0.025                                                      # cross-section height
msection.SetAsRectangularSection(beam_wy, beam_wz)                   # rectangular section
msection.SetYoungModulus(0.01e9)                                     # Young's modulus
msection.SetShearModulus(0.01e9 * 0.3)                               # shear modulus (~0.3 * E)
msection.SetRayleighDamping(0.000)                                   # no Rayleigh damping
msection.SetCentroid(0, 0.02)                                        # centroid offset
msection.SetShearCenter(0, 0.1)                                      # shear center offset
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)              # section rotation

# Add some EULER-BERNOULLI BEAMS:

beam_L = 0.1                                                         # element length

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))           # node at origin
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))      # node at beam_L
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0))) # node at 2*beam_L

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()
belement1.SetNodes(hnode1, hnode2)                                   # connect nodes 1-2
belement1.SetSection(msection)                                       # assign section
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()
belement2.SetNodes(hnode2, hnode3)                                   # connect nodes 2-3
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Apply a force or a torque to a node:
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))                          # force at node 2
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))                     # torque at node 3

# Fix a node to ground using constraints:
mtruss = chrono.ChBody()
mtruss.SetFixed(True)                                                # fixed truss / ground
sys.Add(mtruss)

constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,   # x, y, z constrained
                               True, True, True)   # Rx, Ry, Rz constrained

constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,     # x free, y z constrained
                              False, False, False)   # rotations free

# Add some EULER-BERNOULLI BEAMS (the fast way!)

builder = fea.ChBuilderBeamEuler()                                   # builder helper

# Build first beam from A to B using the builder:
builder.BuildBeam(mesh,                                              # mesh to add nodes/elements to
                  msection,                                          # beam section
                  5,                                                 # number of elements
                  chrono.ChVector3d(0, 0, -0.1),                    # 'A' start point
                  chrono.ChVector3d(0.2, 0, -0.1),                  # 'B' end point
                  chrono.ChVector3d(0, 1, 0))                        # 'Y' up direction

# Fix the last node of the built beam and apply a force to the first node:
builder.GetLastBeamNodes().back().SetFixed(True)                     # fix the back (B end) node
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))  # force on front (A end) node

# Store last beam nodes before the next BuildBeam call (SWIG GC safety)
last_nodes_1 = builder.GetLastBeamNodes()                            # keep reference to avoid GC
node_A_second_beam = last_nodes_1.front()                            # A node for second beam segment

# Add another beam segment using the last node of the previous beam as starting point:
builder.BuildBeam(mesh, msection, 5,
                  node_A_second_beam,                               # 'A' node: last beam's first node
                  chrono.ChVector3d(0.2, 0.1, -0.1),               # 'B' point in space
                  chrono.ChVector3d(0, 1, 0))                        # 'Y' up direction

# We do not want gravity effect on FEA elements in this demo
mesh.SetAutomaticGravity(False)                                      # disable auto-gravity on mesh

# Remember to add the mesh to the system!
sys.Add(mesh)

# Attach FEA visualization shapes
visualizebeamA = chrono.ChVisualShapeFEA(mesh)                       # surface shape for bending moment
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)                        # color scale range
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = chrono.ChVisualShapeFEA(mesh)                       # glyph shape for node coordinate systems
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node coordinate triads
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()                                                      # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()

# Change the solver to the MKL Pardiso, more precise for FEA
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

time_step = 0.001                                                     # timestep for FEA beams
sim_end = 10.0                                                        # simulation end time
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
