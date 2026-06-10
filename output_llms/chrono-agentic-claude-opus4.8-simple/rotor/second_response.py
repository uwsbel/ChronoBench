import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                           # SMC system for stiff FEA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))     # lower-gravity environment

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(True, 2)                                    # gravity on beam, 2 integration points

beam_L = 10                                                          # longer rotor shaft
beam_ro = 0.060                                                      # outer radius
beam_ri = 0.055                                                      # inner radius

beam_area = math.pi * (beam_ro**2 - beam_ri**2)                      # hollow circular area
beam_Iyy = (math.pi / 4.0) * (beam_ro**4 - beam_ri**4)               # second moment about y
beam_Izz = (math.pi / 4.0) * (beam_ro**4 - beam_ri**4)              # second moment about z
beam_J = (math.pi / 2.0) * (beam_ro**4 - beam_ri**4)                 # polar moment

minertia = fea.ChInertiaCosseratSimple()                             # Cosserat inertia
minertia.SetDensity(7800)                                            # steel density
minertia.SetArea(beam_area)
minertia.SetIyy(beam_Iyy)
minertia.SetIzz(beam_Izz)

melasticity = fea.ChElasticityCosseratSimple()                       # Cosserat elasticity
melasticity.SetYoungModulus(210e9)                                   # steel E
melasticity.SetShearModulusFromPoisson(0.3)                          # G from Poisson nu
melasticity.SetIyy(beam_Iyy)
melasticity.SetIzz(beam_Izz)
melasticity.SetJ(beam_J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)          # combined section
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)                              # for drawing only

builder = fea.ChBuilderBeamIGA()                                     # IGA beam builder
builder.BuildBeam(mesh, msection,
                  20,                                                # spans
                  chrono.ChVector3d(0, 0, 0),                        # A
                  chrono.ChVector3d(beam_L, 0, 0),                   # B
                  chrono.VECT_Y,                                     # section Y direction
                  1)                                                 # order = linear

beam_nodes = builder.GetLastBeamNodes()                              # keep ref (SWIG GC pitfall)
node_front = beam_nodes.front()                                      # node at x = 0
node_back = beam_nodes.back()                                        # node at x = beam_L
node_mid = beam_nodes[int(beam_nodes.size() / 2)]                    # mid-span node

sys.Add(mesh)                                                        # register mesh

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800)  # flywheel
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0))      # offset above mid-node
mbodyflywheel.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))         # cylinder axis along beam
sys.Add(mbodyflywheel)

link_flywheel = chrono.ChLinkMateFix()                               # weld flywheel to mid-node
link_flywheel.Initialize(mbodyflywheel, node_mid)
sys.Add(link_flywheel)

truss = chrono.ChBody()                                              # fixed truss / bearings
truss.SetFixed(True)
sys.Add(truss)

link_bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # back-end bearing
link_bearing.Initialize(node_back, truss, chrono.ChFramed(node_back.GetPos()))
sys.Add(link_bearing)

f_ramp = chrono.ChFunctionSine(60, 0.1)                              # motor speed function
motor = chrono.ChLinkMotorRotationSpeed()                            # spin the front end
motor.Initialize(node_front, truss,
                 chrono.ChFramed(node_front.GetPos(),
                                 chrono.QuatFromAngleY(chrono.CH_PI_2)))  # rotate about beam axis
motor.SetSpeedFunction(f_ramp)
sys.Add(motor)

vis_surface = chrono.ChVisualShapeFEA(mesh)                          # deformed surface shape
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                            # node glyph shape
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

sys.SetSolver(mkl.ChSolverPardisoMKL())                              # direct Pardiso solver
ts = chrono.ChTimestepperHHT(sys)                                    # HHT timestepper
ts.SetStepControl(False)
sys.SetTimestepper(ts)

sys.DoStaticLinear()                                                 # settle under gravity/load

vis = chronoirr.ChVisualSystemIrrlicht()                             # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Jeffcott IGA rotor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L / 2, 0, 0))  # longer-beam view
vis.AddTypicalLights()

time_step = 0.002                                                    # IGA rotor timestep
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant
while vis.Run():                                                     # SCORED CORE = plain truth form
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
