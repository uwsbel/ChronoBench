import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                          # SMC system for stiff FEA beams

mesh = fea.ChMesh()                                                 # the FEA mesh holding beam elements and nodes
mesh.SetAutomaticGravity(False)                                    # static/forced response — no FEA self-weight

beam_E = 0.02e10                                                    # Young's modulus [Pa]
beam_density = 1000                                                # density [kg/m^3]

# Euler-Bernoulli beam section (shared by the manual beam and the builder beam)
msection = fea.ChBeamSectionEulerAdvanced()                        # Euler-Bernoulli section
beam_wy = 0.012                                                     # cross-section width along Y [m]
beam_wz = 0.025                                                     # cross-section width along Z [m]
msection.SetAsRectangularSection(beam_wy, beam_wz)                 # rectangular cross section
msection.SetYoungModulus(beam_E)                                  # E
msection.SetShearModulusFromPoisson(0.38)                        # derive G from Poisson nu
msection.SetRayleighDamping(0.01)                                # small Rayleigh damping
msection.SetDensity(beam_density)                                # section density

# --- Manual beam: two xyzrot nodes joined by one Euler element ---
hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))   # node 1 at origin
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0.2, 0, 0))) # node 2 at x = 0.2
mesh.AddNode(hnode1)                                              # register node 1
mesh.AddNode(hnode2)                                              # register node 2

belement1 = fea.ChElementBeamEuler()                             # one Euler-Bernoulli beam element
belement1.SetNodes(hnode1, hnode2)                              # span node 1 -> node 2
belement1.SetSection(msection)                                  # assign the section
mesh.AddElement(belement1)                                     # register the element

# Fix node 1 with a constraint instead of SetFixed (commented-out direct fixing below)
# hnode1.SetFixed(True)
mtruss = chrono.ChBody()                                         # ground truss to anchor the constraint
mtruss.SetFixed(True)                                            # truss is fixed in the world
sys.Add(mtruss)                                                 # add truss to the system

constr_bc = chrono.ChLinkMateGeneric()                          # general constraint fixing node 1 to the truss
constr_bc.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())  # tie node 1 frame to truss
sys.Add(constr_bc)                                             # add the constraint
constr_bc.SetConstrainedCoords(True, True, True,               # tx, ty, tz constrained
                               True, True, True)               # rx, ry, rz constrained

# Apply a force and torque to node 2 of the manual beam
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))                    # external force on node 2 [N]
hnode2.SetTorque(chrono.ChVector3d(0, 0, 0.04))               # external torque on node 2 [Nm]

# --- Builder beam: Euler-Bernoulli beam from point A to point B with 5 elements ---
builder = fea.ChBuilderBeamEuler()                              # helper that auto-creates nodes + elements
builder.BuildBeam(mesh,                                         # target mesh
                  msection,                                     # section to use
                  5,                                            # number of elements
                  chrono.ChVector3d(0, 0, -0.1),               # point A
                  chrono.ChVector3d(0.2, 0, -0.1),             # point B
                  chrono.ChVector3d(0, 1, 0))                  # 'Y' up direction of the section

builder.GetLastBeamNodes().back().SetFixed(True)               # fix the last node of the created beam

builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))  # force on first node of the created beam

sys.Add(mesh)                                                  # register the mesh with the system

# FEA visualization: bending-moment surface field + node coordinate-system glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)                    # surface shape (mesh is a ctor arg)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # color by bending moment Mz
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                    # colorscale range
vis_surface.SetSmoothFaces(True)                             # smooth shading
vis_surface.SetWireframe(False)                             # solid surface
mesh.AddVisualShapeFEA(vis_surface)                         # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                   # glyph shape for node markers
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate-system triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)         # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                       # triad line thickness
vis_glyph.SetSymbolsScale(0.01)                           # triad size
vis_glyph.SetZbufferHide(False)                          # always draw the triads
mesh.AddVisualShapeFEA(vis_glyph)                       # register glyph shape

# MKL Pardiso direct solver (required for stiff Euler beams)
solver = mkl.ChSolverPardisoMKL()                          # Pardiso MKL direct solver
sys.SetSolver(solver)                                     # set as the system solver

# Irrlicht visualization window (Initialize first, then scene elements; NO grid)
vis = chronoirr.ChVisualSystemIrrlicht()                   # create the Irrlicht visual system
vis.AttachSystem(sys)                                     # attach the physical system
vis.SetWindowSize(1024, 768)                            # window size
vis.SetWindowTitle("Euler-Bernoulli beams (FEA)")      # window title
vis.Initialize()                                        # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                         # sky box
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.5), chrono.ChVector3d(0.1, 0, -0.05))  # eye, target
vis.AddTypicalLights()                                 # standard lights

time_step = 1e-3                                        # integration time step [s]
sim_end = 5.0                                           # simulation end time [s]
render_fps = 30.0                                       # review render rate
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
