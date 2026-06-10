import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC system for the FEA cable
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity for FEA scene
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # collision system for cable contact

mesh = fea.ChMesh()                                                    # FEA mesh holding the cable
mesh.SetAutomaticGravity(True)                                        # let gravity act on the elements

# ANCF cable section — thin flexible cable
sec_cable = fea.ChBeamSectionCable()                                  # ANCF cable cross-section
sec_cable.SetDiameter(0.015)                                          # 15 mm diameter
sec_cable.SetYoungModulus(0.01e9)                                    # low E -> very flexible
sec_cable.SetRayleighDamping(0.000)                                  # no structural damping

# Build the cable from node A to node B with the ANCF builder
builder = fea.ChBuilderCableANCF()                                    # ANCF cable beam builder
builder.BuildBeam(mesh, sec_cable, 10,                                # 10 cable elements
                  chrono.ChVector3d(0, 0, -0.1),                      # A — start node
                  chrono.ChVector3d(0.5, 0, -0.1))                    # B — end node

# Keep strong references to the builder node container (SWIG GC pitfall)
cable_nodes = builder.GetLastBeamNodes()                              # cache: hold the node container
cable_elems = builder.GetLastBeamElements()                          # cache: hold the element container
n_nodes = cable_nodes.size()                                          # number of cable nodes

# Apply a small initial force to the free tip node to excite the cable
cable_nodes.back().SetForce(chrono.ChVector3d(0, -0.7, 0))           # downward tip load (N)

# Contact material + node-cloud contact surface so the cable can collide
contact_mat = chrono.ChContactMaterialSMC()                          # SMC contact material for the cable
contact_mat.SetYoungModulus(1e6)                                     # contact stiffness
contact_mat.SetFriction(0.3)                                         # contact friction
contact_mat.SetRestitution(0.0)                                      # no bounce
contact_mat.SetAdhesion(0.0)                                         # no adhesion

contact_cloud = fea.ChContactSurfaceNodeCloud(contact_mat)          # node-cloud contact surface
contact_cloud.AddAllNodes(0.0075)                                   # contact radius around each node
mesh.AddContactSurface(contact_cloud)                              # attach the contact surface

# Hinge one end of the cable to a fixed ground truss
truss = chrono.ChBody()                                               # fixed ground truss
truss.SetFixed(True)                                                  # truss does not move
sys.Add(truss)
hinge = fea.ChLinkNodeFrame()                                         # 3-DOF translational hinge
hinge.Initialize(cable_nodes.front(), truss)                         # pin the first node to the truss
sys.Add(hinge)

sys.Add(mesh)                                                         # register the mesh with the system

# FEA visualization — surface field (axial tension Mx) + node glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)                          # surface/scalar field shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)  # axial force field
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                          # color scale range
vis_surface.SetSmoothFaces(True)                                    # smooth beam surface
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                           # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # nodal position dots
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)    # no field on the glyph shape
vis_glyph.SetSymbolsThickness(0.006)                               # dot thickness
vis_glyph.SetSymbolsScale(0.01)                                    # dot scale
vis_glyph.SetZbufferHide(False)                                    # always draw nodes on top
mesh.AddVisualShapeFEA(vis_glyph)

# Sparse-QR solver + linearized implicit timestepper (ANCF cable)
solver = chrono.ChSolverSparseQR()                                  # direct sparse solver for ANCF
sys.SetSolver(solver)
solver.UseSparsityPatternLearner(True)                              # learn the sparsity pattern
solver.LockSparsityPattern(True)                                   # lock it after learning
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)              # linearized implicit integrator
sys.SetTimestepper(ts)

# Irrlicht visualization window (Initialize first, then scene elements, NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                  # Y-up camera for this scene
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("ANCF Cable Beam")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.2, 0.2, -1.0), chrono.ChVector3d(0.2, -0.2, -0.1))  # eye, target
vis.AddTypicalLights()

time_step = 0.01                                                   # ANCF cable timestep
sim_end = 10.0                                                     # simulation end time
render_fps = 50.0                                                  # target render rate
render_every = max(1, round(1.0 / (render_fps * time_step)))      # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
