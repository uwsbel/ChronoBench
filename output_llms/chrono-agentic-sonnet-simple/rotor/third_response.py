import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Custom motor function with piecewise rotational speed
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)   # MUST call base ctor
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w  = w

    def GetVal(self, x):   # x = time; return speed [rad/s]
        if x < self.T1:
            return self.A1 * math.sin(self.w * x)            # sinusoidal ramp
        elif x < self.T2:
            return self.A1                                    # constant plateau
        elif x < self.T3:
            return self.A1 + (self.A2 - self.A1) * (x - self.T2) / (self.T3 - self.T2)  # linear ramp
        else:
            return self.A2                                    # high-speed steady state

# System and solver
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # Y-up FEA convention
sys.SetSolver(mkl.ChSolverPardisoMKL())                           # Pardiso for stiff IGA beams

# HHT timestepper — canonical-minimal (truth style)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# Rotor shaft dimensions
beam_L   = 1.0         # shaft length [m]
beam_ro  = 0.050       # shaft outer radius [m] — larger for visibility
density  = 7800.0      # steel [kg/m3]
E        = 210e9       # Young's modulus [Pa]
nu       = 0.3         # Poisson ratio

# Cross-section properties (solid circle)
area = math.pi * beam_ro**2
Iyy  = math.pi * beam_ro**4 / 4.0
Izz  = Iyy
J    = Iyy + Izz       # polar moment

# Cosserat section setup
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(density)
minertia.SetArea(area)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(E)
melasticity.SetShearModulusFromPoisson(nu)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ(J)

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)   # visual draw radius (NOT SetAsCircularSection, which overwrites Iyy/Izz/J)

# FEA mesh
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)   # forced rotation: skip mesh auto-gravity

# IGA beam along X axis
n_spans = 10
order   = 3   # cubic IGA

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, msection,
    n_spans,
    chrono.ChVector3d(0, 0, 0),           # A
    chrono.ChVector3d(beam_L, 0, 0),      # B
    chrono.VECT_Y,                        # section Y direction
    order
)

# Keep strong refs to nodes (SWIG GC pitfall)
beam_nodes = builder.GetLastBeamNodes()
nodes = [beam_nodes[i] for i in range(beam_nodes.size())]

sys.Add(mesh)

# FEA visual shapes — add BEFORE vis.Initialize() (required for Irrlicht FEA rendering)
vis_surface = chrono.ChVisualShapeFEA(mesh)                    # coloured deformed surface
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_surface.SetColorscaleMinMax(-0.4, 0.4)
vis_surface.SetSmoothFaces(True)
vis_surface.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_surface)

vis_glyph = chrono.ChVisualShapeFEA(mesh)                      # node coordinate system glyphs
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_glyph.SetSymbolsThickness(0.006)
vis_glyph.SetSymbolsScale(0.01)
vis_glyph.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_glyph)

# Truss (fixed ground)
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetMass(1.0)
truss.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(truss)

# Flywheel at mid-span
flywheel_mass   = 2.0          # [kg]
flywheel_radius = 0.20         # [m]
node_mid = nodes[len(nodes) // 2]

flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
Icyl = 0.5 * flywheel_mass * flywheel_radius**2
flywheel.SetInertiaXX(chrono.ChVector3d(Icyl / 2, Icyl / 2, Icyl))
flywheel.SetPos(node_mid.GetPos())
sys.Add(flywheel)

# Flywheel cylinder visual (disc along shaft X-axis)
cyl_shape = chrono.ChVisualShapeCylinder(flywheel_radius, 0.08)
flywheel.AddVisualShape(cyl_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Weld flywheel to mid-span node
weld = chrono.ChLinkMateFix()
weld.Initialize(node_mid, flywheel)
sys.Add(weld)

# Bearing A — pin at left end (ty, tz, ry, rz constrained)
bearing_A = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing_A.Initialize(nodes[0], truss, False, nodes[0].Frame(), nodes[0].Frame())
sys.Add(bearing_A)

# Bearing B — radial only at right end (ty, tz constrained; axial free)
bearing_B = chrono.ChLinkMateGeneric(False, True, True, False, False, False)
bearing_B.Initialize(nodes[-1], truss, False, nodes[-1].Frame(), nodes[-1].Frame())
sys.Add(bearing_B)

# Motor: prescribed speed at left bearing node
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(nodes[0], truss, chrono.ChFramed(nodes[0].GetPos(), chrono.QUNIT))

# Custom piecewise motor function
A1 = 0.5                      # first speed target [rad/s]
A2 = 3.0                      # high-speed target [rad/s]
T1 = 0.5                      # end of sine ramp [s]
T2 = 1.5                      # end of constant plateau [s]
T3 = 2.5                      # end of linear transition [s]
w  = math.pi / (2.0 * T1)    # sine frequency so sin(w*T1) = 1 -> A1

myfun = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
motor.SetSpeedFunction(myfun)
sys.Add(motor)

# Static pre-solve to initialize
sys.DoStaticLinear()

# Irrlicht — Initialize() FIRST, scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Rotor FEA - Custom Motor Function")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.8, 1.5), chrono.ChVector3d(0.5, 0.0, 0.0))
vis.AddTypicalLights()
vis.BindAll()   # explicitly bind all scene objects including FEA mesh visual shapes

# Simulation parameters
time_step    = 0.002   # IGA rotor timestep [s]
sim_end      = 5.0     # total simulation time [s]
render_fps   = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence


while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
