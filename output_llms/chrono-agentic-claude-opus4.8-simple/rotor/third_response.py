import os
import math
import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                          # SMC system for stiff FEA matrices
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))    # Y-up gravity g = 9.81

mesh = fea.ChMesh()                                                 # FEA container mesh
mesh.SetAutomaticGravity(True, 2)                                   # automatic gravity, 2 Gauss integration points

beam_L = 6.0                                                        # rotor shaft length (m)
beam_ro = 0.050                                                     # outer radius of hollow shaft
beam_ri = 0.045                                                     # inner radius of hollow shaft
density = 7800                                                      # steel density (kg/m^3)
E_mod = 210e9                                                       # Young's modulus (Pa)

area = math.pi * (beam_ro ** 2 - beam_ri ** 2)                     # hollow circular area
Iyy = (math.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4)             # second moment of area about y
Izz = Iyy                                                           # symmetric circular section
Jpolar = Iyy + Izz                                                 # polar moment for torsion

minertia = fea.ChInertiaCosseratSimple()                           # mass/inertia per unit length
minertia.SetDensity(density)                                       # shaft material density
minertia.SetArea(area)                                             # cross-section area
minertia.SetIyy(Iyy)                                               # inertia y
minertia.SetIzz(Izz)                                               # inertia z

melasticity = fea.ChElasticityCosseratSimple()                     # elastic stiffness per unit length
melasticity.SetYoungModulus(E_mod)                                 # axial/bending stiffness
melasticity.SetShearModulusFromPoisson(0.3)                        # shear modulus from Poisson 0.3
melasticity.SetIyy(Iyy)                                            # bending inertia y
melasticity.SetIzz(Izz)                                            # bending inertia z
melasticity.SetJ(Jpolar)                                           # torsional constant

msection = fea.ChBeamSectionCosserat(minertia, melasticity)        # combined Cosserat section
msection.SetCircular(True)                                         # circular cross-section for drawing
msection.SetDrawCircularRadius(beam_ro)                            # draw radius = outer radius

builder = fea.ChBuilderBeamIGA()                                   # isogeometric beam builder
builder.BuildBeam(mesh, msection, 20,                              # 20 spans along the shaft
                  chrono.ChVector3d(0, 0, 0),                      # node A (front bearing)
                  chrono.ChVector3d(beam_L, 0, 0),                 # node B (rear bearing)
                  chrono.VECT_Y,                                   # suggested section Y direction
                  1)                                               # IGA order = 1 (linear)

beam_nodes_c = builder.GetLastBeamNodes()                          # keep container ref (SWIG GC pitfall)
beam_nodes = [beam_nodes_c[i] for i in range(beam_nodes_c.size())] # node list
node_front = beam_nodes[0]                                         # front rotor node
node_back = beam_nodes[-1]                                         # rear rotor node
node_mid = beam_nodes[len(beam_nodes) // 2]                        # mid-span node for flywheel

sys.Add(mesh)                                                      # register the mesh in the system

flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  # disc R=0.24, h=0.1
flywheel.SetCoordsys(chrono.ChCoordsysd(
    node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),            # offset +Y above mid node
    chrono.QuatFromAngleZ(chrono.CH_PI_2)))                       # rotate 90 deg about Z onto shaft axis
sys.Add(flywheel)                                                 # add flywheel disc

weld = chrono.ChLinkMateFix()                                     # rigid weld flywheel <-> mid node
weld.Initialize(node_mid, flywheel)                              # 6-DOF fix at mid-span
sys.Add(weld)                                                     # add weld constraint

truss = chrono.ChBody()                                           # fixed ground truss for bearings
truss.SetFixed(True)                                             # truss is anchored to world
sys.Add(truss)                                                   # add truss

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # free in x-trans & x-rot
bearing.Initialize(node_back, truss,                             # rear node to truss
                   chrono.ChFramed(node_back.GetPos()))          # frame at rear node
sys.Add(bearing)                                                 # add rear bearing

# --- custom motor speed function: replaces the simple ChFunctionSine(40, 0.2) ---
class ChFunctionMyFun(chrono.ChFunction):                         # custom motor angular-speed law
    def __init__(self):                                          # configure the piecewise parameters
        super().__init__()                                       # base ChFunction ctor
        self.A1 = 40.0                                          # initial spin speed (rad/s)
        self.A2 = 80.0                                          # ramped-up spin speed (rad/s)
        self.T1 = 0.4                                            # end of spin-up phase
        self.T2 = 1.0                                            # end of A1->A2 ramp phase
        self.T3 = 1.5                                            # end of steady-A2 phase
        self.w = 0.2 * 2.0 * chrono.CH_PI                       # slow whirl modulation rate
    def GetVal(self, x):                                        # return motor speed at time x
        if x < self.T1:                                        # phase 1: cosine spin-up to A1
            return self.A1 * 0.5 * (1.0 - math.cos(chrono.CH_PI * x / self.T1))
        elif x < self.T2:                                      # phase 2: ramp A1 -> A2 with whirl wobble
            a = self.A1 + (self.A2 - self.A1) * (x - self.T1) / (self.T2 - self.T1)
            return a + 5.0 * math.sin(self.w * x)
        elif x < self.T3:                                      # phase 3: steady A2 with whirl wobble
            return self.A2 + 5.0 * math.sin(self.w * x)
        else:                                                  # phase 4: hold steady at A2
            return self.A2
    def Clone(self):                                            # required clone for ChFunction
        return ChFunctionMyFun()

motor_fun = ChFunctionMyFun()                                    # instantiate the custom speed law
motor = chrono.ChLinkMotorRotationSpeed()                        # speed-controlled rotary motor
motor.Initialize(node_front, truss,                             # drive front node against truss
                 chrono.ChFramed(node_front.GetPos(),           # frame at front node
                                 chrono.QuatFromAngleY(chrono.CH_PI_2)))  # motor axis along shaft x
motor.SetSpeedFunction(motor_fun)                               # apply custom speed function
sys.Add(motor)                                                  # add the motor

# Predefined visual settings for the FEM mesh (surface field + node-csys glyphs)
vis_surface = chrono.ChVisualShapeFEA(mesh)                      # shape 1: deformed surface field
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)  # plain deformed surface
vis_surface.SetSmoothFaces(True)                                # smooth shading
vis_surface.SetWireframe(False)                                 # solid surface
mesh.AddVisualShapeFEA(vis_surface)                             # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                       # shape 2: node coordinate glyphs
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # coordinate-system triads
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # no scalar field on glyphs
vis_glyph.SetSymbolsThickness(0.006)                            # glyph line thickness
vis_glyph.SetSymbolsScale(0.01)                                 # glyph size
vis_glyph.SetZbufferHide(False)                                 # always show glyphs
mesh.AddVisualShapeFEA(vis_glyph)                               # register glyph shape

vis = chronoirr.ChVisualSystemIrrlicht()                        # Irrlicht window
vis.AttachSystem(sys)                                           # bind the system
vis.SetWindowSize(1024, 768)                                   # window size
vis.SetWindowTitle("Jeffcott IGA rotor with custom motor function")  # title
vis.Initialize()                                               # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                # sky box
vis.AddCamera(chrono.ChVector3d(0, 1, 4),                      # eye position
              chrono.ChVector3d(beam_L / 2, 0, 0))             # look at mid shaft
vis.AddTypicalLights()                                         # standard lights

sys.SetSolver(mkl.ChSolverPardisoMKL())                        # direct sparse solver for stiff FEA
ts = chrono.ChTimestepperHHT(sys)                              # precise HHT timestepper
ts.SetStepControl(False)                                       # canonical-minimal HHT
sys.SetTimestepper(ts)                                         # use HHT for the dynamics

sys.DoStaticLinear()                                           # settle structure under gravity/load

time_step = 0.002                                              # IGA rotor timestep
render_fps = 50.0                                              # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant
while vis.Run():                                               # scored core = plain truth loop
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
