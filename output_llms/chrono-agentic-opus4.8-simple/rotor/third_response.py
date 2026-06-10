import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as mkl

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity

# custom motor speed profile: piecewise ramp/hold/sinusoid (replaces the simple sine)
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)                              # MUST call base ctor
        self.A1 = A1                                                  # first plateau speed (rad/s)
        self.A2 = A2                                                  # second plateau speed (rad/s)
        self.T1 = T1                                                  # end of initial ramp
        self.T2 = T2                                                  # end of first hold
        self.T3 = T3                                                  # end of second ramp
        self.w = w                                                    # sinusoid pulsation
    def GetVal(self, x):                                              # x = time -> motor speed
        if x < self.T1:                                              # linear ramp up to A1
            return self.A1 * x / self.T1
        elif x < self.T2:                                            # hold at A1
            return self.A1
        elif x < self.T3:                                            # ramp from A1 to A2
            return self.A1 + (self.A2 - self.A1) * (x - self.T2) / (self.T3 - self.T2)
        else:                                                        # A2 with superposed sinusoid
            return self.A2 + 0.2 * self.A2 * math.sin(self.w * (x - self.T3))

beam_L = 6.0                                                          # rotor shaft length (m)
beam_ro = 0.050                                                       # outer radius of hollow shaft (m)
beam_ri = 0.045                                                       # inner radius of hollow shaft (m)
density = 7800                                                        # steel density (kg/m^3)
E_mod = 210e9                                                         # Young's modulus (Pa)

area = math.pi * (beam_ro ** 2 - beam_ri ** 2)                       # annulus cross-section area
Iyy = (math.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4)                # second moment about y
Izz = Iyy                                                             # symmetric circular section
Jpolar = Iyy + Izz                                                   # polar moment (torsion)

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(False)                                      # forced response: no FEA self-gravity

minertia = fea.ChInertiaCosseratSimple()                            # per-section inertia
minertia.SetDensity(density)                                         # shaft material density
minertia.SetArea(area)                                               # cross-section area
minertia.SetIyy(Iyy)                                                 # bending inertia y
minertia.SetIzz(Izz)                                                 # bending inertia z

melasticity = fea.ChElasticityCosseratSimple()                      # per-section elasticity
melasticity.SetYoungModulus(E_mod)                                  # stiffness
melasticity.SetShearModulusFromPoisson(0.3)                        # derive G from Poisson nu
melasticity.SetIyy(Iyy)                                             # bending stiffness y
melasticity.SetIzz(Izz)                                             # bending stiffness z
melasticity.SetJ(Jpolar)                                            # torsional stiffness

msection = fea.ChBeamSectionCosserat(minertia, melasticity)        # combined Cosserat section
msection.SetCircular(True)                                          # circular cross-section flag
msection.SetDrawCircularRadius(beam_ro)                            # visual radius only (keeps Iyy/Izz/J)

builder = fea.ChBuilderBeamIGA()                                    # IGA (large-rotation) beam builder
builder.BuildBeam(mesh, msection,
                  30,                                               # number of spans
                  chrono.ChVector3d(0, 0, 0),                       # node A (left end)
                  chrono.ChVector3d(beam_L, 0, 0),                  # node B (right end)
                  chrono.VECT_Y,                                    # suggested section Y direction
                  3)                                               # cubic order

beam_nodes = builder.GetLastBeamNodes()                            # keep ref (SWIG GC guard)
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]          # materialize node list
node_left = nodes[0]                                               # driven end
node_right = nodes[-1]                                             # bearing end
node_mid = nodes[len(nodes) // 2]                                 # mid-span (flywheel mount)

sys.Add(mesh)                                                      # register mesh with system

# flywheel disc rigidly welded to the mid-span node (Jeffcott rotor mass)
flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.05, 7800, True)  # r, h, density
flywheel.SetPos(node_mid.GetPos())                                # place at mid node
flywheel.SetRot(chrono.QuatFromAngleZ(chrono.CH_PI_2))            # disc plane perpendicular to shaft (X)
marker = chrono.ChVisualShapeBox(0.06, 0.10, 0.06)               # rim marker to make spin visible
marker.SetColor(chrono.ChColor(0.9, 0.1, 0.1))                  # red index mark
flywheel.AddVisualShape(marker, chrono.ChFramed(chrono.ChVector3d(0, 0.20, 0)))  # at the disc rim
sys.Add(flywheel)

weld = chrono.ChLinkMateFix()                                      # rigid 6-DOF weld
weld.Initialize(node_mid, flywheel)                               # tie node to flywheel body
sys.Add(weld)

truss = chrono.ChBody()                                            # fixed reference frame
truss.SetFixed(True)                                               # ground
sys.Add(truss)

# bearing at the right end: allow shaft spin about X, block lateral whirl
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # free: tx, rx
bearing.Initialize(node_right, truss, False, node_right.Frame(), node_right.Frame())
sys.Add(bearing)

# prescribed-speed motor drives the left node about the shaft (X) axis
motor = chrono.ChLinkMotorRotationSpeed()                          # full motor-link (no extra revolute)
q_xaxis = chrono.QuatFromAngleY(chrono.CH_PI_2)                    # map motor local +Z onto world +X
motor.Initialize(node_left, truss, chrono.ChFramed(node_left.GetPos(), q_xaxis))
motor_fun = ChFunctionMyFun(A1=20.0, A2=60.0, T1=1.0, T2=2.0, T3=4.0, w=8.0)  # custom speed profile
motor.SetSpeedFunction(motor_fun)                                 # install the custom function
sys.Add(motor)

# FEA visualization: deformed surface field + node coordinate-system glyphs
vis_surface = chrono.ChVisualShapeFEA(mesh)                       # mesh is a ctor arg (9.0.0)
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)  # plain deformed surface
vis_surface.SetSmoothFaces(True)                                 # smooth shading
vis_surface.SetWireframe(False)                                  # solid surface
mesh.AddVisualShapeFEA(vis_surface)                              # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                        # second shape: node glyphs
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.02)                             # triad line thickness
vis_glyph.SetSymbolsScale(0.10)                                # triad size (visible spin)
vis_glyph.SetZbufferHide(False)                                # always draw glyphs
mesh.AddVisualShapeFEA(vis_glyph)                              # register glyph shape

sys.SetSolver(mkl.ChSolverPardisoMKL())                        # direct solver for stiff beam stiffness

ts = chrono.ChTimestepperHHT(sys)                             # implicit HHT for stiff dynamics
ts.SetStepControl(False)                                     # canonical-minimal HHT
sys.SetTimestepper(ts)                                       # install timestepper

sys.DoStaticLinear()                                         # settle the structure before dynamics

vis = chronoirr.ChVisualSystemIrrlicht()                     # Irrlicht render window
vis.AttachSystem(sys)                                        # bind the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)           # Y-up scene
vis.SetWindowSize(1280, 720)                                # window resolution
vis.SetWindowTitle("IGA Cosserat Rotor")                    # window title
vis.Initialize()                                            # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                             # sky box
vis.AddCamera(chrono.ChVector3d(3, 0.8, -2.2), chrono.ChVector3d(3, 0, 0))  # close on the flywheel
vis.AddTypicalLights()                                     # standard lighting

time_step = 0.002                                          # IGA rotor timestep
sim_end = 8.0                                              # simulation duration (s)
render_fps = 50.0                                          # frames per second for review
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
