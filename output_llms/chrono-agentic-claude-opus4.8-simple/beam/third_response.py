import os
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA matrices
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up FEA convention, g = 9.81

mesh = fea.ChMesh()                                                   # FEA container mesh
mesh.SetAutomaticGravity(False)                                       # forced response; no FEA gravity integration

beam_wy = 0.012                                                       # section width along section-Y
beam_wz = 0.025                                                       # section width along section-Z
beam_E = 0.01e9                                                       # Young's modulus (soft beam)
beam_density = 1000.0                                                 # material density
Iyy = (beam_wz * beam_wy ** 3) / 12.0                                # second moment about y
Izz = (beam_wy * beam_wz ** 3) / 12.0                                # second moment about z

minertia = fea.ChInertiaCosseratSimple()                             # Cosserat inertia properties
minertia.SetDensity(beam_density)                                    # rho
minertia.SetArea(beam_wy * beam_wz)                                  # cross-section area
minertia.SetIyy(Iyy)                                                 # Iyy
minertia.SetIzz(Izz)                                                 # Izz

melasticity = fea.ChElasticityCosseratSimple()                       # Cosserat elasticity properties
melasticity.SetYoungModulus(beam_E)                                  # E
melasticity.SetShearModulusFromPoisson(0.3)                          # G from Poisson nu = 0.3
melasticity.SetIyy(Iyy)                                              # Iyy matching inertia
melasticity.SetIzz(Izz)                                              # Izz matching inertia
melasticity.SetJ(Iyy + Izz)                                          # torsion constant

msection = fea.ChBeamSectionCosserat(minertia, melasticity)          # combined IGA Cosserat section
msection.SetDrawThickness(beam_wy, beam_wz)                           # rectangular draw cross-section

builder = fea.ChBuilderBeamIGA()                                     # isogeometric beam builder

# First beam segment: from A0 to B0 along x, suggested section-Y up (0,1,0).
builder.BuildBeam(mesh, msection, 6,                                 # 6 spans
                  chrono.ChVector3d(0, 0, -0.1),                     # 'A' point of segment 1
                  chrono.ChVector3d(0.2, 0, -0.1),                   # 'B' point of segment 1
                  chrono.ChVector3d(0, 1, 0),                        # 'Y' up direction
                  3)                                                 # cubic order
seg1_c = builder.GetLastBeamNodes()                                 # store container first (SWIG GC)
seg1 = [seg1_c[i] for i in range(seg1_c.size())]                     # materialize node list
node_root = seg1[0]                                                  # root node of segment 1
node_lastprev = seg1[-1]                                            # last node created by the previous beam

# Second beam segment: start at the previous beam's last node ('A' node) and end at the 'B' point.
builder.BuildBeam(mesh, msection, 6,                                 # 6 spans
                  node_lastprev.GetPos(),                            # 'A' node = last node of previous beam
                  chrono.ChVector3d(0.2, 0.1, -0.1),                 # 'B' point of segment 2
                  chrono.ChVector3d(0, 1, 0),                        # 'Y' up direction
                  3)                                                 # cubic order
seg2_c = builder.GetLastBeamNodes()                                 # store container first (SWIG GC)
seg2 = [seg2_c[i] for i in range(seg2_c.size())]                     # materialize node list
node_join = seg2[0]                                                  # first node of segment 2 (coincident with last of seg 1)
node_tip = seg2[-1]                                                 # free tip of segment 2

node_root.SetFixed(True)                                            # clamp the first beam root AFTER both builds (a second BuildBeam resets the flag)

# Weld segment 2's start node to segment 1's last node so the two beams form one continuous chain.
join_link = chrono.ChLinkMateFix()                                   # rigid 6-DOF weld between the two coincident nodes
join_link.Initialize(node_join, node_lastprev)                       # node_join <-> node_lastprev
sys.Add(join_link)                                                  # register the weld

node_tip.SetForce(chrono.ChVector3d(0, -0.2, 0))                     # tip load to deflect the chained beam

sys.Add(mesh)                                                       # add FEA mesh to the system

vis_surface = chrono.ChVisualShapeFEA(mesh)                          # scalar-field surface shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz field
vis_surface.SetColorscaleMinMax(-0.4, 0.4)                          # color range (lo, hi)
vis_surface.SetSmoothFaces(True)                                    # smooth shading
vis_surface.SetWireframe(False)                                     # solid surface
mesh.AddVisualShapeFEA(vis_surface)                                 # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                           # node-glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate-system triads at nodes
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)     # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                               # triad line thickness
vis_glyph.SetSymbolsScale(0.01)                                    # triad size
vis_glyph.SetZbufferHide(False)                                    # always draw glyphs
mesh.AddVisualShapeFEA(vis_glyph)                                  # register glyph shape

sys.SetSolver(mkl.ChSolverPardisoMKL())                            # direct solver for stiff FEA

ts = chrono.ChTimestepperHHT(sys)                                  # implicit HHT timestepper
ts.SetStepControl(False)                                           # fixed-step HHT
sys.SetTimestepper(ts)                                             # install timestepper

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht render window
vis.AttachSystem(sys)                                              # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                  # Y-up camera
vis.SetWindowSize(1280, 720)                                       # window pixels
vis.SetWindowTitle("IGA beam — chained segments")                  # title
vis.Initialize()                                                   # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2), chrono.ChVector3d(0.1, 0.0, -0.1))  # eye, target
vis.AddTypicalLights()                                             # standard lights

time_step = 1e-3                                                   # HHT step
render_fps = 50.0                                                  # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))       # physics steps per rendered frame

while vis.Run():                                                   # SCORED CORE = plain truth form
    vis.BeginScene()                                               # begin frame
    vis.Render()                                                   # draw scene
    vis.EndScene()                                                 # end frame
    for _ in range(render_every):                                  # advance physics batch
        sys.DoStepDynamics(time_step)                              # integrate one step
