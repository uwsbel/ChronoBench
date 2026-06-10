import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA matrices

mesh = fea.ChMesh()                                                   # FEA mesh holding nodes + elements
mesh.SetAutomaticGravity(False)                                       # static/forced response — no FEA self-gravity

# Beam section: Euler-Bernoulli advanced rectangular section
beam_wy = 0.012                                                       # section width along y (m)
beam_wz = 0.025                                                       # section width along z (m)
msection = fea.ChBeamSectionEulerAdvanced()                          # advanced Euler-Bernoulli section
msection.SetAsRectangularSection(beam_wy, beam_wz)                    # rectangular cross-section
msection.SetYoungModulus(0.01e9)                                      # E = 0.01 GPa
msection.SetShearModulus(0.01e9 * 0.3)                                # G = 0.3 * E
msection.SetRayleighDamping(0.000)                                    # no structural damping
msection.SetCentroid(0, 0.02)                                         # offset of centroid in section
msection.SetShearCenter(0, 0.1)                                       # offset of shear center
msection.SetSectionRotation(45 * chrono.CH_RAD_TO_DEG)               # rotate the section orientation

# Truss: a fixed rigid body the beam constraints attach to
truss = chrono.ChBody()                                              # ground/truss reference body
truss.SetFixed(True)                                                 # immovable
sys.Add(truss)

# Manually-built beam: 3 ChNodeFEAxyzrot at x = 0, 0.1, 0.2 and 2 ChElementBeamEuler
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))      # node 1 (root end)
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.1, 0, 0)))    # node 2 (middle)
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0)))    # node 3 (tip)
mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()                                 # element between node1 and node2
belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)
mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()                                 # element between node2 and node3
belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)
mesh.AddElement(belement2)

# Applied loads on the manual beam
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))                          # transverse force on the middle node
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))                     # bending torque at the tip

# Node-fixing approach: instead of hnode1.SetFixed(True), use ChLinkMateGeneric constraints.
# hnode1.SetFixed(True)                                               # replaced by a constraint below
# Fully fix node 3 to the truss (all 6 DOF)
constr_a = chrono.ChLinkMateGeneric()                                # constraint for node 3 ↔ truss
constr_a.Initialize(hnode3, truss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_a)
constr_a.SetConstrainedCoords(True, True, True, True, True, True)    # lock x,y,z + 3 rotations

# Constrain node 1 only in y,z translation (leave x and rotations free)
constr_b = chrono.ChLinkMateGeneric()                                # constraint for node 1 ↔ truss
constr_b.Initialize(hnode1, truss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_b)
constr_b.SetConstrainedCoords(False, True, True, False, False, False)  # lock only y,z translation

# Euler-Bernoulli beam built with the ChBuilderBeamEuler helper object
builder = fea.ChBuilderBeamEuler()                                   # helper that meshes a beam between two points
builder.BuildBeam(mesh,                                              # target mesh
                  msection,                                          # reuse the Euler section
                  5,                                                 # number of elements
                  chrono.ChVector3d(0, 0, -0.1),                     # point A
                  chrono.ChVector3d(0.2, 0, -0.1),                   # point B
                  chrono.ChVector3d(0, 1, 0))                        # 'Y' up direction
# Fix the last node of the built beam
builder.GetLastBeamNodes().back().SetFixed(True)                     # clamp the far end of the built beam
# Apply a force to the first node of the built beam section
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))  # downward tip force

sys.Add(mesh)                                                        # register the mesh with the system

# Visualization shapes for the FEA mesh (two-shape pattern)
vis_beam_mz = chrono.ChVisualShapeFEA(mesh)                          # surface field: bending moment Mz
vis_beam_mz.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam_mz.SetColorscaleMinMax(-0.4, 0.4)                           # color range for Mz
vis_beam_mz.SetSmoothFaces(True)
vis_beam_mz.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_mz)

vis_beam_nodes = chrono.ChVisualShapeFEA(mesh)                       # node coordinate-system glyphs
vis_beam_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_beam_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_nodes.SetSymbolsThickness(0.006)
vis_beam_nodes.SetSymbolsScale(0.01)
vis_beam_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_nodes)

# Pardiso MKL direct solver (stiff FEA matrices)
sys.SetSolver(mkl.ChSolverPardisoMKL())                             # MKL Pardiso direct solver

# Irrlicht visualization window
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Euler-Bernoulli beam FEA")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()

time_step = 0.001                                                    # physics timestep
render_fps = 50.0                                                    # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():                                                     # plain truth-form render loop
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
