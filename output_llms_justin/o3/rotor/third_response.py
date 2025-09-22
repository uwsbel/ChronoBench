###############################################################
# Jeffcott rotor – IGA beam example (corrected & extended)
###############################################################

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------
# 1.  Custom motor-speed function
# ----------------------------------------------------------------
class ChFunctionMyFun(chrono.ChFunction):
    """
    Custom piece-wise motor speed law:
        For 0      <= t <  T1               ->  ω = A1
        For T1     <= t <  T1+T2            ->  linear ramp  A1 → A2
        For T1+T2  <= t <  T1+T2+T3         ->  sinusoid     A2*sin(w*t')
        For t >= T1+T2+T3                   ->  ω = 0
    """

    def __init__(self, A1, A2, T1, T2, T3, w):
        super().__init__()
        self.A1, self.A2 = A1, A2
        self.T1, self.T2, self.T3 = T1, T2, T3
        self.w = w

    # Chrono looks for Get_y (older) or GetVal (newer). We provide both.
    def Get_y(self, x):       # For backward compatibility
        return self.GetVal(x)

    def GetVal(self, x):      # Chrono >= 7.0.x
        if x < self.T1:
            return self.A1

        if x < self.T1 + self.T2:
            return self.A1 + (self.A2 - self.A1) * (x - self.T1) / self.T2

        if x < self.T1 + self.T2 + self.T3:
            t_prime = x - self.T1 - self.T2
            return self.A2 * m.sin(self.w * t_prime)

        return 0.0


# ----------------------------------------------------------------
# 2.  Physical system, mesh and material
# ----------------------------------------------------------------
sys = chrono.ChSystemSMC()

# (optional) more precise HHT stepper – uncomment if wanted
# hht = chrono.ChTimestepperHHT(sys)
# hht.SetAlpha(-0.2)
# hht.SetMaxiters(50)
# sys.SetTimestepper(hht)

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)              # 2 Gauss pts/elem for cubic IGA
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))  # global gravity

beam_L  = 6.0
beam_ro = 0.050
beam_ri = 0.045
PI      = m.pi

# --- Cosserat section -------------------------------------------------
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(PI * (beam_ro**2 - beam_ri**2))
Iyy = Izz = (PI / 4.0) * (beam_ro**4 - beam_ri**4)
minertia.SetIyy(Iyy)
minertia.SetIzz(Izz)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(Iyy)
melasticity.SetIzz(Izz)
melasticity.SetJ((PI / 2.0) * (beam_ro**4 - beam_ri**4))

section = fea.ChBeamSectionCosserat(minertia, melasticity)
section.SetCircular(True)
section.SetDrawCircularRadius(beam_ro)

# --- Build IGA beam ---------------------------------------------------
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                  section,
                  20,                                   # number of spans
                  chrono.ChVector3d(0, 0, 0),          # start
                  chrono.ChVector3d(beam_L, 0, 0),     # end
                  chrono.VECT_Y,                       # reference Y
                  1)                                   # polynomial order

# handy references to the node list
nodes      = builder.GetLastBeamNodes()
node_start = nodes[0]
node_end   = nodes[-1]
node_mid   = nodes[len(nodes)//2]

# ----------------------------------------------------------------
# 3.  Fly-wheel attached at mid-span
# ----------------------------------------------------------------
flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis.Y, 0.24, 0.1, 7800)  # R, h, ρ
flywheel.SetCoordsys(chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                                        chrono.Q_from_AngAxis(PI/2.0, chrono.VECT_Z)))
sys.Add(flywheel)

fix_fw = chrono.ChLinkMateFix()
fix_fw.Initialize(node_mid, flywheel)
sys.Add(fix_fw)

# ----------------------------------------------------------------
# 4.  Bearings & motor
# ----------------------------------------------------------------
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# rear bearing ---------------------------------------------------
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(node_end,
                   truss,
                   chrono.ChFrameD(node_end.GetPos()))
sys.Add(bearing)

# front motor/bearing -------------------------------------------
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node_start,
                 truss,
                 chrono.ChFrameD(node_start.GetPos(),
                                 chrono.Q_from_AngAxis(PI/2.0, chrono.VECT_Y)))
sys.Add(motor)

# use the custom function
my_speed_fun = ChFunctionMyFun(A1=0.0, A2=40.0, T1=0.5, T2=1.0, T3=2.0, w=6.0)
motor.SetMotorFunction(my_speed_fun)

# ----------------------------------------------------------------
# 5.  FEA visualization
# ----------------------------------------------------------------
vis_beam_surf = chrono.ChVisualShapeFEA(mesh)
vis_beam_surf.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beam_surf.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam_surf)

vis_beam_nodes = chrono.ChVisualShapeFEA(mesh)
vis_beam_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_beam_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_nodes.SetSymbolsThickness(0.006)
vis_beam_nodes.SetSymbolsScale(0.01)
vis_beam_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_nodes)

# ----------------------------------------------------------------
# 6.  Irrlicht visualisation
# ----------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott rotor – IGA beam (PyChrono)')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L/2, 0, 0))
vis.AddTypicalLights()

# ----------------------------------------------------------------
# 7.  Solver & analysis
# ----------------------------------------------------------------
sys.SetSolver(mkl.ChSolverPardisoMKL())
sys.DoStaticLinear()

# ----------------------------------------------------------------
# 8.  Time integration loop
# ----------------------------------------------------------------
timestep = 0.002
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(timestep)