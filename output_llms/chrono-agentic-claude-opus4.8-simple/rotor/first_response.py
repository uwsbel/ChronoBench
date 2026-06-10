import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA matrices
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity, g = 9.81

mesh = fea.ChMesh()                                                   # FEA container for the IGA beam
mesh.SetAutomaticGravity(True, 2)                                     # gravity on the beam, 2 integration points

beam_L = 6.0                                                          # rotor shaft length [m]
beam_ro = 0.050                                                       # outer radius [m]
beam_ri = 0.045                                                       # inner radius (hollow shaft) [m]

beam_area = math.pi * (beam_ro**2 - beam_ri**2)                       # hollow circular cross-section area
beam_Iyy = (math.pi / 4.0) * (beam_ro**4 - beam_ri**4)               # second moment about y
beam_Izz = beam_Iyy                                                   # symmetric → Izz = Iyy
beam_J = beam_Iyy + beam_Izz                                          # polar moment

minertia = fea.ChInertiaCosseratSimple()                             # Cosserat inertia properties
minertia.SetDensity(7800)                                            # steel density [kg/m^3]
minertia.SetArea(beam_area)                                          # cross-section area
minertia.SetIyy(beam_Iyy)                                            # bending inertia y
minertia.SetIzz(beam_Izz)                                            # bending inertia z

melasticity = fea.ChElasticityCosseratSimple()                       # Cosserat elasticity properties
melasticity.SetYoungModulus(210e9)                                   # steel E [Pa]
melasticity.SetShearModulusFromPoisson(0.3)                          # G from Poisson nu = 0.3
melasticity.SetIyy(beam_Iyy)                                         # match section inertia y
melasticity.SetIzz(beam_Izz)                                         # match section inertia z
melasticity.SetJ(beam_J)                                             # torsional constant

msection = fea.ChBeamSectionCosserat(minertia, melasticity)          # combined IGA section
msection.SetCircular(True)                                           # circular cross-section
msection.SetDrawCircularRadius(beam_ro)                              # draw radius (no Iyy/Izz/J overwrite)

builder = fea.ChBuilderBeamIGA()                                     # isogeometric beam builder
builder.BuildBeam(mesh, msection,
                  20,                                                # number of spans
                  chrono.ChVector3d(0, 0, 0),                        # node A
                  chrono.ChVector3d(beam_L, 0, 0),                   # node B
                  chrono.VECT_Y,                                     # suggested section Y direction
                  1)                                                 # order 1 (linear)

beam_nodes = builder.GetLastBeamNodes()                              # keep container ref (SWIG GC pitfall)
n_nodes = beam_nodes.size()
node_front = beam_nodes[0]                                           # driven end (x = 0)
node_back = beam_nodes[n_nodes - 1]                                  # bearing end (x = beam_L)
node_mid = beam_nodes[n_nodes // 2]                                  # flywheel attachment (x = beam_L/2)

sys.Add(mesh)                                                        # register the FEA mesh

flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  # disk R=0.24, h=0.1, steel
flywheel.SetCoordsys(chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                                        chrono.QuatFromAngleZ(chrono.CH_PI_2)))  # +Y offset, rotate 90 about Z
sys.Add(flywheel)                                                   # add the flywheel body

link_flywheel = chrono.ChLinkMateFix()                              # rigid weld flywheel ↔ mid-node
link_flywheel.Initialize(flywheel, node_mid)                        # fix to the central beam node
sys.Add(link_flywheel)

truss = chrono.ChBody()                                             # fixed reference truss
truss.SetFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # bearing: free x-trans + x-rot
bearing.Initialize(node_back, truss,
                   chrono.ChFramed(node_back.GetPos()))             # at the back node
sys.Add(bearing)

motor = chrono.ChLinkMotorRotationSpeed()                          # rotational motor at the driven end
motor.Initialize(node_front, truss,
                 chrono.ChFramed(node_front.GetPos(), chrono.QuatFromAngleY(chrono.CH_PI_2)))  # spin about beam x-axis
motor.SetSpeedFunction(chrono.ChFunctionSine(40, 0.2))             # sinusoidal speed: amp 40, freq 0.2
sys.Add(motor)

vis_surface = chrono.ChVisualShapeFEA(mesh)                        # surface field shape (deformed shaft)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                          # node coordinate-system glyphs
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.SetSolver(mkl.ChSolverPardisoMKL())                            # direct sparse solver for stiff FEA
ts = chrono.ChTimestepperHHT(sys)                                  # HHT implicit timestepper
ts.SetStepControl(False)
sys.SetTimestepper(ts)

sys.DoStaticLinear()                                               # settle the structure under gravity/load

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott IGA rotor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))  # eye, look at shaft center
vis.AddTypicalLights()

time_step = 0.002                                                 # IGA rotor timestep
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))     # untagged render-cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
