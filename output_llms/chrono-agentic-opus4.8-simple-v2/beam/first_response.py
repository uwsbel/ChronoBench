import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA beams
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity, g = 9.81

mesh = fea.ChMesh()                                                    # FEA mesh container for nodes/elements
mesh.SetAutomaticGravity(True)                                        # let gravity load the beam elements

beam_L = 2.0                                                          # total beam length (m)
beam_diam = 0.010                                                     # circular cross-section diameter (m)
n_elements = 16                                                       # number of Euler beam elements

msection = fea.ChBeamSectionEulerAdvanced()                           # Euler-Bernoulli advanced beam section
msection.SetAsCircularSection(beam_diam)                              # circular cross section of given diameter
msection.SetDensity(2700)                                             # aluminium density (kg/m^3)
msection.SetYoungModulus(73e9)                                        # aluminium Young's modulus (Pa)
msection.SetShearModulusFromPoisson(0.3)                              # derive G from Poisson nu = 0.3
msection.SetRayleighDamping(0.000)                                    # no Rayleigh structural damping

builder = fea.ChBuilderBeamEuler()                                    # helper that meshes a straight beam
builder.BuildBeam(
    mesh, msection, n_elements,                                      # mesh, section, element count
    chrono.ChVector3d(0, 0, 0),                                      # beam start point A
    chrono.ChVector3d(beam_L, 0, 0),                                 # beam end point B
    chrono.ChVector3d(0, 1, 0),                                      # suggested section Y (up) direction
)

beam_nodes = builder.GetLastBeamNodes()                               # keep a strong ref (SWIG GC guard)
node_root = beam_nodes.front()                                       # clamped root node at A
node_tip = beam_nodes.back()                                         # free tip node at B

truss = chrono.ChBody()                                               # fixed truss the beam roots to
truss.SetFixed(True)                                                 # ground reference, immovable
sys.Add(truss)                                                       # register the truss body

constr = chrono.ChLinkMateGeneric()                                   # general node-to-ground constraint
constr.Initialize(node_root, truss, False, node_root.Frame(), node_root.Frame())  # weld at the root frame
constr.SetConstrainedCoords(True, True, True,                        # lock tx, ty, tz
                            True, True, True)                        # lock rx, ry, rz (full clamp)
sys.Add(constr)                                                      # register the clamp constraint

node_tip.SetForce(chrono.ChVector3d(0, -2, 0))                        # downward tip load (N)
node_tip.SetTorque(chrono.ChVector3d(0, 0, 0.2))                      # small bending torque about Z (Nm)

msection2 = fea.ChBeamSectionEulerAdvanced()                          # second, thinner beam section
msection2.SetAsCircularSection(0.006)                                # 6 mm circular cross section
msection2.SetDensity(2700)                                           # aluminium density (kg/m^3)
msection2.SetYoungModulus(73e9)                                      # aluminium Young's modulus (Pa)
msection2.SetShearModulusFromPoisson(0.3)                            # derive G from Poisson nu = 0.3
msection2.SetRayleighDamping(0.000)                                  # no Rayleigh structural damping

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(node_tip.GetPos()))     # first node of the manual extension beam
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(node_tip.GetPos() + chrono.ChVector3d(0.5, 0, 0)))  # mid node
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(node_tip.GetPos() + chrono.ChVector3d(1.0, 0, 0)))  # far node
mesh.AddNode(hnode1)                                                 # register node 1
mesh.AddNode(hnode2)                                                 # register node 2
mesh.AddNode(hnode3)                                                 # register node 3

belem1 = fea.ChElementBeamEuler()                                    # first manual Euler beam element
belem1.SetNodes(hnode1, hnode2)                                      # span nodes 1->2
belem1.SetSection(msection2)                                         # assign the thinner section
mesh.AddElement(belem1)                                              # register element 1

belem2 = fea.ChElementBeamEuler()                                    # second manual Euler beam element
belem2.SetNodes(hnode2, hnode3)                                      # span nodes 2->3
belem2.SetSection(msection2)                                         # assign the thinner section
mesh.AddElement(belem2)                                              # register element 2

hnode3.SetForce(chrono.ChVector3d(0, -1, 0))                          # extra downward load on the far node (N)

link_beams = chrono.ChLinkMateGeneric()                              # couple the first beam tip to the extension
link_beams.Initialize(node_tip, hnode1, False, node_tip.Frame(), hnode1.Frame())  # weld their frames
link_beams.SetConstrainedCoords(True, True, True,                   # lock tx, ty, tz
                                True, True, True)                   # lock rx, ry, rz (rigid joint)
sys.Add(link_beams)                                                 # register the inter-beam joint

sys.Add(mesh)                                                        # register the FEA mesh with the system

vis_surface = chrono.ChVisualShapeFEA(mesh)                           # surface/scalar field shape (mesh ctor arg)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # colour by bending moment Mz
vis_surface.SetColorscaleMinMax(-4.0, 4.0)                           # colour scale low/high (two scalars)
vis_surface.SetSmoothFaces(True)                                    # smooth shaded beam surface
vis_surface.SetWireframe(False)                                     # solid (not wireframe) rendering
mesh.AddVisualShapeFEA(vis_surface)                                  # attach the surface visual shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                            # node-glyph shape for markers
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # draw node coordinate triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)     # no scalar field on the glyph shape
vis_glyph.SetSymbolsThickness(0.006)                               # glyph line thickness
vis_glyph.SetSymbolsScale(0.01)                                    # glyph symbol scale
vis_glyph.SetZbufferHide(False)                                    # keep glyphs visible through geometry
mesh.AddVisualShapeFEA(vis_glyph)                                   # attach the glyph visual shape

solver = mkl.ChSolverPardisoMKL()                                    # direct sparse solver for stiff beams
sys.SetSolver(solver)                                               # iterative solvers diverge on FEA

ts = chrono.ChTimestepperHHT(sys)                                    # implicit HHT timestepper
ts.SetStepControl(False)                                            # canonical-minimal HHT form
sys.SetTimestepper(ts)                                              # install the timestepper

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht real-time render window
vis.AttachSystem(sys)                                              # bind the physical system to the view
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                  # Y-up camera convention (matches gravity)
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetWindowTitle("FEA Euler beam")                               # window title
vis.Initialize()                                                  # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # add the pychrono logo
vis.AddSkyBox()                                                   # standard sky box backdrop
vis.AddCamera(chrono.ChVector3d(1.5, 0.8, -3.4),                  # camera eye position
              chrono.ChVector3d(1.5, -0.6, 0))                    # camera look-at target (mid beam)
vis.AddTypicalLights()                                            # standard two-light setup

time_step = 1e-3                                                    # stiff-beam time step (s)
sim_end = 5.0                                                       # total simulated duration (s)
render_fps = 50.0                                                   # target review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))       # untagged render-cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
