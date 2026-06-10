import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # FEA uses SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))      # lower-gravity environment, Y-up

beam_L = 10                                                           # beam length (units)
beam_ro = 0.060                                                       # outer radius
beam_ri = 0.055                                                       # inner radius
n_elements = 6                                                        # number of IGA spans

mesh = fea.ChMesh()                                                   # FEA mesh container
mesh.SetAutomaticGravity(True)                                        # gravity acts on the beam

area = chrono.CH_PI * (beam_ro ** 2 - beam_ri ** 2)                   # hollow circular area
Iyy = chrono.CH_PI / 4.0 * (beam_ro ** 4 - beam_ri ** 4)             # second moment of area
Izz = Iyy                                                             # axisymmetric shaft
Jpolar = Iyy + Izz                                                    # polar moment

minertia = fea.ChInertiaCosseratSimple()                             # mass distribution
minertia.SetDensity(7800)                                            # steel
minertia.SetArea(area)                                               # cross-section area
minertia.SetIyy(Iyy)                                                 # bending inertia y
minertia.SetIzz(Izz)                                                 # bending inertia z

melasticity = fea.ChElasticityCosseratSimple()                       # stiffness model
melasticity.SetYoungModulus(210e9)                                   # steel E
melasticity.SetShearModulusFromPoisson(0.3)                          # G from Poisson
melasticity.SetArea(area)                                            # cross-section area
melasticity.SetIyy(Iyy)                                              # bending stiffness y
melasticity.SetIzz(Izz)                                              # bending stiffness z
melasticity.SetJ(Jpolar)                                             # torsional stiffness

msection = fea.ChBeamSectionCosserat(minertia, melasticity)          # combined section
msection.SetCircular(True)                                           # circular cross-section
msection.SetDrawCircularRadius(beam_ro)                              # draw radius (does not overwrite I/J)

builder = fea.ChBuilderBeamIGA()                                     # IGA beam builder
builder.BuildBeam(mesh, msection, n_elements,                        # build the rotor shaft
                  chrono.ChVector3d(0, 0, 0),                        # node A (start)
                  chrono.ChVector3d(beam_L, 0, 0),                   # node B (end)
                  chrono.VECT_Y,                                     # section Y direction
                  3)                                                 # cubic order

beam_nodes = builder.GetLastBeamNodes()                              # keep strong ref (SWIG GC)
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]            # all shaft nodes
node_first = nodes[0]                                                # driven end node
node_last = nodes[-1]                                                # far bearing node
node_mid = nodes[len(nodes) // 2]                                    # mid-span node for flywheel

sys.Add(mesh)                                                        # register mesh

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # flywheel disk
mbodyflywheel.SetPos(node_mid.GetPos())                              # place at mid-span node
mbodyflywheel.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))          # disk axis along beam X
sys.Add(mbodyflywheel)                                               # register flywheel

weld = chrono.ChLinkMateFix()                                        # rigidly weld flywheel to mid node
weld.Initialize(node_mid, mbodyflywheel)                             # node <-> body
sys.Add(weld)                                                        # register weld

truss = chrono.ChBody()                                              # fixed truss / bearing housing
truss.SetFixed(True)                                                 # immovable
sys.Add(truss)                                                       # register truss

bearing_A = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # free rot about X, free trans X
bearing_A.Initialize(node_first, truss, False, node_first.Frame(), node_first.Frame())  # near bearing
sys.Add(bearing_A)                                                   # register bearing A

bearing_B = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # free rot about X
bearing_B.Initialize(node_last, truss, False, node_last.Frame(), node_last.Frame())  # far bearing
sys.Add(bearing_B)                                                   # register bearing B

f_ramp = chrono.ChFunctionSine(60, 0.1)                              # sine torque: ampl=60 Nm, freq=0.1 Hz

motor = chrono.ChLinkMotorRotationTorque()                           # torque motor on the shaft end
motor.Initialize(node_first, truss, chrono.ChFramed(node_first.GetPos(), chrono.QUNIT))  # node vs truss
motor.SetTorqueFunction(f_ramp)                                      # apply the sine torque
sys.Add(motor)                                                       # register motor

vis_surface = chrono.ChVisualShapeFEA(mesh)                          # deformed surface field
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)  # plain deformed surface
vis_surface.SetSmoothFaces(True)                                     # smooth shading
vis_surface.SetWireframe(False)                                      # solid
mesh.AddVisualShapeFEA(vis_surface)                                  # attach surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                            # node markers
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)      # no scalar field on glyph
vis_glyph.SetSymbolsThickness(0.006)                                # triad thickness
vis_glyph.SetSymbolsScale(0.01)                                     # triad scale
vis_glyph.SetZbufferHide(False)                                     # always show
mesh.AddVisualShapeFEA(vis_glyph)                                   # attach glyph shape

sys.SetSolver(mkl.ChSolverPardisoMKL())                             # direct solver for stiff beam

ts = chrono.ChTimestepperHHT(sys)                                   # implicit HHT integrator
ts.SetStepControl(False)                                           # canonical-minimal form
sys.SetTimestepper(ts)                                             # install timestepper

sys.DoStaticLinear()                                               # settle under gravity / load

vis = chronoirr.ChVisualSystemIrrlicht()                          # render window
vis.AttachSystem(sys)                                             # attach physical system
vis.SetWindowSize(1280, 720)                                     # window resolution
vis.SetWindowTitle("Jeffcott Rotor")                            # window title
vis.Initialize()                                                # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                 # sky box
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # better view of longer beam
vis.AddTypicalLights()                                         # standard lights

time_step = 0.002                                              # IGA rotor timestep
sim_end = 10.0                                                # simulation duration
render_fps = 50.0                                            # review video cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))  # physics steps per frame (untagged)
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
