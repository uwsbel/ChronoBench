import math
import os
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # FEA scenes use SMC
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity

mesh = fea.ChMesh()                                                   # FEA container mesh
mesh.SetAutomaticGravity(False)                                       # rotor is a forced response

beam_wy = 0.012                                                       # shaft outer wall scale
beam_wz = 0.012                                                       # shaft outer wall scale
beam_L = 6                                                            # total rotor length (m)
beam_ro = 0.050                                                       # shaft outer radius
beam_ri = 0.045                                                       # shaft inner radius (hollow)

area = math.pi * (beam_ro ** 2 - beam_ri ** 2)                        # cross-section area
Iyy = (math.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4)                 # 2nd moment about y
Izz = (math.pi / 4.0) * (beam_ro ** 4 - beam_ri ** 4)                 # 2nd moment about z
Jpolar = Iyy + Izz                                                    # polar inertia (torsion)

minertia = fea.ChInertiaCosseratSimple()                             # lumped beam inertia
minertia.SetDensity(7800)                                            # steel density
minertia.SetArea(area)                                               # cross-section area
minertia.SetIyy(Iyy)                                                 # inertia about y
minertia.SetIzz(Izz)                                                 # inertia about z

melasticity = fea.ChElasticityCosseratSimple()                       # beam elasticity
melasticity.SetYoungModulus(210e9)                                   # steel E
melasticity.SetShearModulusFromPoisson(0.3)                          # G from Poisson nu
melasticity.SetIyy(Iyy)                                              # bending inertia y
melasticity.SetIzz(Izz)                                              # bending inertia z
melasticity.SetJ(Jpolar)                                            # torsional constant

msection = fea.ChBeamSectionCosserat(minertia, melasticity)          # combined Cosserat section
msection.SetCircular(True)                                           # circular cross-section
msection.SetDrawCircularRadius(beam_ro)                              # draw outer radius

builder = fea.ChBuilderBeamIGA()                                     # IGA (Cosserat) beam builder
builder.BuildBeam(mesh, msection,                                    # build into the mesh
                  20,                                                # number of spans
                  chrono.ChVector3d(0, 0, 0),                        # node A
                  chrono.ChVector3d(beam_L, 0, 0),                   # node B
                  chrono.VECT_Y,                                     # suggested section Y dir
                  3)                                                 # cubic order

beam_nodes = builder.GetLastBeamNodes()                              # keep strong ref (SWIG GC)
node_first = beam_nodes.front()                                      # rotor input end node
node_last = beam_nodes.back()                                       # rotor far-bearing end node
node_mid = beam_nodes[int(beam_nodes.size() / 2)]                    # mid-span node (flywheel)

flywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.05, 7800, True)  # Jeffcott disk
flywheel.SetPos(node_mid.GetPos())                                   # place disk at mid node
mrot = chrono.ChQuaterniond()                                       # align disk axis to shaft
mrot.SetFromAngleAxis(chrono.CH_PI_2, chrono.VECT_Z)                 # rotate disk plane to x-axis
flywheel.SetRot(mrot)                                                # orient flywheel
sys.Add(flywheel)                                                    # add disk to system

weld = chrono.ChLinkMateFix()                                        # rigidly weld disk to node
weld.Initialize(node_mid, flywheel)                                  # node <-> flywheel
sys.Add(weld)                                                        # add weld constraint

truss = chrono.ChBody()                                             # fixed support truss
truss.SetFixed(True)                                                # ground reference
sys.Add(truss)                                                       # add truss

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)  # radial bearing (free x-trans + x-rot)
bearing.Initialize(node_last, truss, False, node_last.Frame(), node_last.Frame())  # far end bearing
sys.Add(bearing)                                                    # add bearing constraint

motor = chrono.ChLinkMotorRotationSpeed()                           # speed-driven motor
motor.Initialize(node_first, truss, chrono.ChFramed(node_first.GetPos(), chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_Y)))  # drive about beam x
sys.Add(motor)                                                      # add motor

A1 = 0.20                                                           # first ramp speed amplitude
A2 = 0.40                                                           # steady speed amplitude
T1 = 0.5                                                            # end of first ramp
T2 = 1.0                                                            # end of oscillatory phase
T3 = 1.25                                                           # end of final ramp
w = 8                                                              # oscillation angular frequency


class ChFunctionMyFun(chrono.ChFunction):                           # custom piecewise motor speed
    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)                            # MUST call base ctor
        self.A1 = A1                                                # store amplitudes/times
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):                                            # x = time -> motor speed (rad/s)
        if x < self.T1:                                            # phase 1: smooth ramp-up
            return self.A1 * (1.0 - math.cos(chrono.CH_PI * x / self.T1)) / 2.0
        elif x < self.T2:                                          # phase 2: oscillatory ride
            return self.A1 + (self.A2 - self.A1) * (x - self.T1) / (self.T2 - self.T1) \
                + 0.1 * math.sin(self.w * (x - self.T1))
        elif x < self.T3:                                          # phase 3: ramp to steady
            return self.A2 * (1.0 - math.cos(chrono.CH_PI * (self.T3 - x) / (self.T3 - self.T2))) / 2.0 + self.A2 * 0.0 + self.A2
        else:                                                     # phase 4: constant steady speed
            return self.A2

    def Clone(self):                                              # SWIG requires a clone
        return ChFunctionMyFun(self.A1, self.A2, self.T1, self.T2, self.T3, self.w)


myfun = ChFunctionMyFun(A1, A2, T1, T2, T3, w)                      # instantiate custom function
motor.SetSpeedFunction(myfun)                                       # drive motor with custom speed

sys.Add(mesh)                                                       # register the FEA mesh

vis_surface = chrono.ChVisualShapeFEA(mesh)                         # shape 1: deformed surface field
vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)  # plain deformed surface
vis_surface.SetSmoothFaces(True)                                   # smooth shading
vis_surface.SetWireframe(False)                                    # solid faces
mesh.AddVisualShapeFEA(vis_surface)                                # register surface shape

vis_glyph = chrono.ChVisualShapeFEA(mesh)                          # shape 2: node coordinate glyphs
vis_glyph.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)    # no scalar field on glyphs
vis_glyph.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # node coordinate systems
vis_glyph.SetSymbolsThickness(0.006)                              # glyph line thickness
vis_glyph.SetSymbolsScale(0.01)                                  # glyph size
vis_glyph.SetZbufferHide(False)                                  # always draw glyphs
mesh.AddVisualShapeFEA(vis_glyph)                                # register glyph shape

solver = mkl.ChSolverPardisoMKL()                                # direct solver for stiff beam
sys.SetSolver(solver)                                            # attach Pardiso MKL

ts = chrono.ChTimestepperHHT(sys)                               # precise HHT timestepper
ts.SetStepControl(False)                                        # canonical-minimal HHT
sys.SetTimestepper(ts)                                          # attach HHT timestepper

sys.DoStaticLinear()                                            # settle structure pre-dynamics

vis = chronoirr.ChVisualSystemIrrlicht()                       # Irrlicht render window
vis.AttachSystem(sys)                                          # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)             # Y-up camera
vis.SetWindowSize(1280, 720)                                  # window resolution
vis.SetWindowTitle("Jeffcott Rotor — custom motor function")  # window title
vis.Initialize()                                             # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                              # sky box
vis.AddCamera(chrono.ChVector3d(1, 1, 6), chrono.ChVector3d(3, 0, 0))  # eye, look-at mid shaft
vis.AddTypicalLights()                                       # standard lights

time_step = 0.002                                            # IGA rotor timestep
sim_end = 2.0                                                # total simulated time
render_fps = 50.0                                           # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
