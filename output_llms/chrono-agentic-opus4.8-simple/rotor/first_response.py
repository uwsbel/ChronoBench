import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # FEA uses SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))     # Y-up gravity

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(False)                                     # forced-response rotor, no FEA gravity
refs = []                                                           # keep strong refs (SWIG GC)

beam_L = 6.0                                                        # rotor shaft length (m)
beam_ro = 0.050                                                     # shaft outer radius (m)
density = 7800.0                                                    # steel density
E = 210e9                                                           # Young's modulus (Pa)
area = math.pi * beam_ro * beam_ro                                  # circular section area
Iyy = (math.pi / 4.0) * beam_ro**4                                 # second moment of area
Izz = Iyy                                                           # circular -> equal
J = Iyy + Izz                                                       # polar moment

minertia = fea.ChInertiaCosseratSimple()                           # mass/inertia of the section
minertia.SetDensity(density)                                        # steel
minertia.SetArea(area)                                              # cross-section area
minertia.SetIyy(Iyy)                                               # inertia y
minertia.SetIzz(Izz)                                               # inertia z

melasticity = fea.ChElasticityCosseratSimple()                     # elastic stiffness of the section
melasticity.SetYoungModulus(E)                                      # E
melasticity.SetShearModulusFromPoisson(0.3)                        # G from Poisson nu
melasticity.SetIyy(Iyy)                                            # bending stiffness y
melasticity.SetIzz(Izz)                                            # bending stiffness z
melasticity.SetJ(J)                                               # torsional stiffness

msection = fea.ChBeamSectionCosserat(minertia, melasticity)        # IGA Cosserat section
msection.SetCircular(True)                                          # mark circular
msection.SetDrawCircularRadius(beam_ro)                            # draw radius (no Iyy/Izz overwrite)
refs += [minertia, melasticity, msection]                          # retain

builder = fea.ChBuilderBeamIGA()                                   # IGA beam builder
builder.BuildBeam(mesh, msection,
                  20,                                              # number of spans
                  chrono.ChVector3d(0, 0, 0),                      # node A (left end)
                  chrono.ChVector3d(beam_L, 0, 0),                 # node B (right end)
                  chrono.VECT_Y,                                   # suggested section Y
                  3)                                               # cubic order
refs.append(builder)                                              # retain builder

beam_nodes = builder.GetLastBeamNodes()                            # keep container (SWIG GC)
n_nodes = beam_nodes.size()                                        # total beam nodes
node_first = beam_nodes.front()                                    # driven end
node_mid = beam_nodes[n_nodes // 2]                                # center node (flywheel)
refs += [beam_nodes, node_first, node_mid]                         # retain nodes

fly_r = 0.24                                                       # flywheel radius (m)
fly_t = 0.06                                                       # flywheel thickness (m)
fly_rho = 7800.0                                                   # flywheel density
fly_vol = math.pi * fly_r * fly_r * fly_t                         # disc volume
fly_mass = fly_rho * fly_vol                                       # disc mass
flywheel = chrono.ChBody()                                         # flywheel rigid body
flywheel.SetMass(fly_mass)                                         # set mass
flywheel.SetInertiaXX(chrono.ChVector3d(                           # disc inertia
    0.25 * fly_mass * fly_r**2 + (1.0/12.0) * fly_mass * fly_t**2, # Ixx
    0.5 * fly_mass * fly_r**2,                                     # Iyy (spin axis = beam X)
    0.25 * fly_mass * fly_r**2 + (1.0/12.0) * fly_mass * fly_t**2))# Izz
flywheel.SetPos(node_mid.GetPos())                                # at the mid node
fly_cyl = chrono.ChVisualShapeCylinder(fly_r, fly_t)              # disc visual
flywheel.AddVisualShape(fly_cyl,                                   # rotate to align axis with beam X
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleZ(chrono.CH_PI_2)))
sys.Add(flywheel)                                                 # add flywheel

weld = chrono.ChLinkMateFix()                                      # weld flywheel to mid node
weld.Initialize(node_mid, flywheel)                               # rigidly fix
sys.Add(weld)                                                     # add weld
refs.append(weld)                                                # retain

truss = chrono.ChBody()                                            # fixed support / bearing housing
truss.SetFixed(True)                                              # ground
sys.Add(truss)                                                    # add truss

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # radial bearing at right end
bearing.Initialize(beam_nodes.back(), truss, False,               # node B to ground
                   beam_nodes.back().Frame(), beam_nodes.back().Frame())
sys.Add(bearing)                                                  # add bearing
refs += [truss, bearing, beam_nodes.back()]                      # retain

motor = chrono.ChLinkMotorRotationSpeed()                          # speed motor at driven end
motor.Initialize(node_first, truss,                               # between node A and ground
    chrono.ChFramed(node_first.GetPos(),                          # frame: spin about beam X
                    chrono.QuatFromAngleY(chrono.CH_PI_2)))
motor.SetSpeedFunction(chrono.ChFunctionConst(math.pi))          # ~pi rad/s spin rate
sys.Add(motor)                                                    # add motor
refs.append(motor)                                               # retain

sys.Add(mesh)                                                     # register mesh

vis_surface = chrono.ChVisualShapeFEA(mesh)                       # surface field shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)  # deformed surface
vis_surface.SetSmoothFaces(True)                                 # smooth
vis_surface.SetWireframe(False)                                  # solid
mesh.AddVisualShapeFEA(vis_surface)                              # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                        # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                            # triad thickness
vis_glyph.SetSymbolsScale(0.01)                                # triad scale
vis_glyph.SetZbufferHide(False)                                # always visible
mesh.AddVisualShapeFEA(vis_glyph)                              # register glyph shape
refs += [vis_surface, vis_glyph]                              # retain

solver = mkl.ChSolverPardisoMKL()                              # direct solver for stiff beam
sys.SetSolver(solver)                                          # set solver

ts = chrono.ChTimestepperHHT(sys)                              # implicit HHT timestepper
ts.SetStepControl(False)                                      # canonical-minimal
sys.SetTimestepper(ts)                                        # set timestepper

sys.DoStaticLinear()                                          # settle structure once

vis = chronoirr.ChVisualSystemIrrlicht()                      # Irrlicht window
vis.AttachSystem(sys)                                         # attach physics
vis.SetWindowSize(1280, 720)                                  # window size
vis.SetWindowTitle("Jeffcott Rotor - IGA Beam")              # title
vis.Initialize()                                             # Initialize first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                              # sky
vis.AddCamera(chrono.ChVector3d(3, 1.2, -3), chrono.ChVector3d(3, 0, 0))  # eye, target
vis.AddTypicalLights()                                       # lights

time_step = 0.002                                            # IGA rotor timestep
sim_end = 10.0                                               # duration (s)
render_fps = 50.0                                            # frames per second
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
