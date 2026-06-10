import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA beams
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity g = 9.81

beam_L = 1.0                                                          # buckling column length (m)
beam_ro = 0.050 / 2                                                   # outer radius of the hollow tube
beam_ri = 0.045 / 2                                                   # inner radius of the hollow tube
density = 7800                                                        # steel density (kg/m^3)
E_mod = 210e9                                                         # Young's modulus (Pa)
n_beams = 6                                                           # number of IGA spans along the column

mesh = fea.ChMesh()                                                   # FEA container mesh
mesh.SetAutomaticGravity(False)                                       # forced buckling response, no FEA gravity
sys.Add(mesh)                                                         # register the mesh with the system

area = math.pi * (beam_ro ** 2 - beam_ri ** 2)                       # tube cross-section area
Iyy = (math.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4)               # second moment about y
Izz = Iyy                                                            # symmetric tube => Izz = Iyy
Jpol = Iyy + Izz                                                      # polar moment for torsion

minertia = fea.ChInertiaCosseratSimple()                            # Cosserat inertia model
minertia.SetDensity(density)                                         # mass density
minertia.SetArea(area)                                               # section area
minertia.SetIyy(Iyy)                                                # bending inertia y
minertia.SetIzz(Izz)                                                # bending inertia z

melasticity = fea.ChElasticityCosseratSimple()                      # Cosserat elasticity model
melasticity.SetYoungModulus(E_mod)                                  # axial/bending stiffness modulus
melasticity.SetShearModulusFromPoisson(0.31)                        # G derived from Poisson ratio (steel)
melasticity.SetIyy(Iyy)                                             # bending inertia y
melasticity.SetIzz(Izz)                                             # bending inertia z
melasticity.SetJ(Jpol)                                             # torsional constant

msection = fea.ChBeamSectionCosserat(minertia, melasticity)         # combine into a Cosserat section
msection.SetCircular(True)                                          # circular tube cross-section
msection.SetDrawCircularRadius(beam_ro)                            # render radius (does not touch Iyy/Izz/J)

builder = fea.ChBuilderBeamIGA()                                    # isogeometric beam builder
builder.BuildBeam(mesh, msection,                                   # build along the column
                  n_beams,                                          # spans
                  chrono.ChVector3d(0, 0, 0),                       # node A (base)
                  chrono.ChVector3d(beam_L, 0, 0),                  # node B (top), column laid along +X
                  chrono.VECT_Y,                                    # suggested section Y direction
                  3)                                               # IGA order = cubic

beam_nodes = builder.GetLastBeamNodes()                             # keep strong ref (SWIG GC guard)
node_base = beam_nodes.front()                                     # base node of the column
node_top = beam_nodes.back()                                       # loaded/top node of the column

Pcr = math.pi ** 2 * E_mod * Iyy / (2.0 * beam_L) ** 2             # Euler critical load (fixed-free column)

truss = chrono.ChBody()                                            # fixed reference truss
truss.SetFixed(True)                                             # truss is anchored to ground
sys.Add(truss)                                                   # add the truss body

constr_base = chrono.ChLinkMateGeneric()                          # constraint clamping the base node to truss
constr_base.Initialize(node_base, truss, False, node_base.Frame(), node_base.Frame())
constr_base.SetConstrainedCoords(True, True, True, True, True, True)   # clamp all 6 DOF at the base
sys.Add(constr_base)                                            # add the base clamp

constr_top = chrono.ChLinkMateGeneric()                           # constraint guiding the loaded top node
constr_top.Initialize(node_top, truss, False, node_top.Frame(), node_top.Frame())
constr_top.SetConstrainedCoords(False, False, True, False, False, False)   # lock z so buckling stays planar (XY)
sys.Add(constr_top)                                            # add the top guide constraint


class CompressionLoad(chrono.ChFunction):                         # custom load-profile motor function
    def __init__(self):                                          # initialize the custom function
        chrono.ChFunction.__init__(self)                        # MUST call the base constructor
    def GetVal(self, x):                                        # x = time, returns load factor of Pcr
        return 1.5 * min(1.0, x / 3.0)                          # ramp from 0 to 1.5*Pcr over 3 s
    def Clone(self):                                            # C++ side clones the function
        return CompressionLoad()                               # return a fresh instance


load_fun = CompressionLoad()                                    # keep a strong Python reference

crank = chrono.ChBody()                                          # auxiliary motorized crank body
crank.SetMass(0.3)                                             # crank mass
crank.SetPos(chrono.ChVector3d(0, -0.4, 0))                   # offset beside the column base
sys.Add(crank)                                                # add the crank body
crank.AddVisualShape(chrono.ChVisualShapeCylinder(0.015, 0.25))   # visualize the crank arm


class CrankAngle(chrono.ChFunction):                             # custom angle-profile motor function
    def __init__(self):                                         # initialize the custom function
        chrono.ChFunction.__init__(self)                       # MUST call the base constructor
    def GetVal(self, x):                                       # x = time, returns crank angle (rad)
        return 0.5 * chrono.CH_PI * x * x / 8.0                # quadratic spin-up profile
    def Clone(self):                                           # C++ side clones the function
        return CrankAngle()                                   # return a fresh instance


crank_fun = CrankAngle()                                       # keep a strong Python reference
motor = chrono.ChLinkMotorRotationAngle()                       # angle-prescribed motor (full motor-link)
motor.Initialize(crank, truss, chrono.ChFramed(chrono.ChVector3d(0, -0.4, 0)))   # drive crank vs truss
motor.SetAngleFunction(crank_fun)                             # apply the custom angle profile
sys.Add(motor)                                                # add the driving motor

vis_beam = chrono.ChVisualShapeFEA(mesh)                        # surface/scalar field visual shape
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)   # color by bending moment Mz
vis_beam.SetColorscaleMinMax(-500, 500)                       # color scale (lo, hi) in N*m
vis_beam.SetSmoothFaces(True)                                # smooth shaded beam surface
vis_beam.SetWireframe(False)                                # solid, not wireframe
mesh.AddVisualShapeFEA(vis_beam)                            # register the surface shape

vis_nodes = chrono.ChVisualShapeFEA(mesh)                      # glyph visual shape for node frames
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)   # coordinate-system triads
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)         # no field on the glyphs
vis_nodes.SetSymbolsThickness(0.006)                        # triad line thickness
vis_nodes.SetSymbolsScale(0.01)                            # triad size
vis_nodes.SetZbufferHide(False)                            # always draw the triads
mesh.AddVisualShapeFEA(vis_nodes)                          # register the glyph shape

solver = mkl.ChSolverPardisoMKL()                            # direct sparse solver for stiff beams
sys.SetSolver(solver)                                       # attach the Pardiso MKL solver

ts = chrono.ChTimestepperHHT(sys)                           # implicit HHT timestepper
ts.SetStepControl(False)                                   # canonical-minimal HHT (no adaptive step)
sys.SetTimestepper(ts)                                     # attach the timestepper

vis = chronoirr.ChVisualSystemIrrlicht()                    # Irrlicht render window
vis.AttachSystem(sys)                                      # bind the system to the visualizer
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)         # Y-up world for the camera
vis.SetWindowSize(1280, 720)                             # window resolution
vis.SetWindowTitle("Beam buckling")                      # window title
vis.Initialize()                                         # create the device (FIRST)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # PyChrono logo
vis.AddSkyBox()                                          # sky box backdrop
vis.AddCamera(chrono.ChVector3d(0.5, 0.4, 1.8), chrono.ChVector3d(0.5, 0.3, 0.0))   # eye, target
vis.AddTypicalLights()                                   # standard two-light setup

time_step = 1e-3                                         # stiff-beam timestep
sim_end = 8.0                                            # total simulated time (s)
render_fps = 50.0                                        # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        P = load_fun.GetVal(sys.GetChTime()) * Pcr                  # ramped axial compressive load
        node_top.SetForce(chrono.ChVector3d(-P, 0.02 * P, 0))       # compress + small lateral imperfection
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
