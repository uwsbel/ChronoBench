import os
import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()                                            # SMC system for stiff FEA beams
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # Y-up gravity (FEA convention)

beam_L = 1.0                                                          # column length (m), oriented along Y
beam_wy = 0.012                                                       # rectangular section width y (m)
beam_wz = 0.025                                                       # rectangular section width z (m)
density = 1000.0                                                      # material density (kg/m^3)
E = 0.02e10                                                           # Young modulus (Pa) — slender column
n_elements = 12                                                       # IGA beam spans
beam_order = 3                                                        # cubic IGA order

minertia = fea.ChInertiaCosseratSimple()                             # mass/inertia of the section
minertia.SetAsRectangularSection(beam_wy, beam_wz, density)          # rectangular cross-section inertia

melasticity = fea.ChElasticityCosseratSimple()                       # elastic stiffness of the section
melasticity.SetYoungModulus(E)                                       # axial/bending stiffness
melasticity.SetShearModulusFromPoisson(0.3)                          # G from Poisson ratio
melasticity.SetAsRectangularSection(beam_wy, beam_wz)                # rectangular cross-section stiffness

msection = fea.ChBeamSectionCosserat(minertia, melasticity)          # combined Cosserat section
msection.SetDrawThickness(beam_wy, beam_wz)                          # visual section size

mesh = fea.ChMesh()                                                  # FEA mesh container
mesh.SetAutomaticGravity(False)                                      # static buckling response, no FEA self-gravity
sys.Add(mesh)                                                        # register the mesh

builder = fea.ChBuilderBeamIGA()                                     # IGA Cosserat beam builder
builder.BuildBeam(mesh, msection, n_elements,                        # build the vertical column
                  chrono.ChVector3d(0, 0, 0),                        # A: bottom end
                  chrono.ChVector3d(0, beam_L, 0),                   # B: top end
                  chrono.VECT_Z,                                     # suggested section Y direction
                  beam_order)                                        # cubic order

beam_nodes = builder.GetLastBeamNodes()                              # keep a strong ref (SWIG GC)
node_bottom = beam_nodes.front()                                     # clamped base node
node_top = beam_nodes.back()                                         # driven top node

truss = chrono.ChBody()                                              # fixed ground truss
truss.SetFixed(True)                                                 # anchor it in space
sys.Add(truss)                                                       # register the truss

constr_bottom = chrono.ChLinkMateGeneric()                           # clamp the base node to the truss
constr_bottom.Initialize(node_bottom, truss, False,                  # node A, truss, all 6 DOF
                         node_bottom.Frame(), node_bottom.Frame())
constr_bottom.SetConstrainedCoords(True, True, True, True, True, True)  # fully clamp the base
sys.Add(constr_bottom)                                              # register the clamp

slider = chrono.ChBody()                                             # moving carriage at the top
slider.SetMass(1.0)                                                  # small carriage mass
slider.SetPos(chrono.ChVector3d(0, beam_L, 0))                       # placed at the column top
sys.Add(slider)                                                      # register the carriage

constr_top = chrono.ChLinkMateGeneric()                             # connect top node to the carriage
constr_top.Initialize(node_top, slider, False,                      # node B, carriage
                      node_top.Frame(), node_top.Frame())
constr_top.SetConstrainedCoords(True, True, True, True, True, True)  # weld top node to carriage
sys.Add(constr_top)                                                # register the top constraint

guide = chrono.ChLinkMotorLinearPosition()                          # prismatic motor guiding the carriage
guide.Initialize(slider, truss,                                     # carriage moves relative to truss
                 chrono.ChFramed(chrono.ChVector3d(0, beam_L, 0),   # motor frame at the top
                                 chrono.QuatFromAngleX(-chrono.CH_PI_2)))  # motion axis along world Y


class CompressionFunction(chrono.ChFunction):                       # custom motor function: smooth downward press
    def __init__(self, stroke, t_press):
        chrono.ChFunction.__init__(self)                           # init the base ChFunction
        self.stroke = stroke                                       # signed downward travel (m, negative = down)
        self.t_press = t_press                                     # time to complete the press (s)

    def GetVal(self, x):                                          # commanded motor displacement at time x
        if x >= self.t_press:                                     # hold after the press completes
            return self.stroke
        s = 0.5 * (1.0 - math.cos(math.pi * x / self.t_press))    # smooth 0->1 cosine ramp
        return self.stroke * s                                    # scale to the full stroke

    def GetDer(self, x):                                          # analytic derivative (HHT needs it)
        if x >= self.t_press:                                     # zero rate once held
            return 0.0
        return self.stroke * 0.5 * (math.pi / self.t_press) * math.sin(math.pi * x / self.t_press)

    def Clone(self):                                              # required for SWIG-owned copies
        return CompressionFunction(self.stroke, self.t_press)


press_fun = CompressionFunction(-0.12 * beam_L, 2.0)               # press the top down by 12% of length over 2 s
guide.SetMotorFunction(press_fun)                                  # drive the carriage with the custom function
sys.Add(guide)                                                     # register the motor

node_mid = beam_nodes[beam_nodes.size() // 2]                      # mid-span node for the buckling trigger
node_mid.SetForce(chrono.ChVector3d(0, 0, 0.6))                    # small lateral perturbation to seed buckling

vis_beam = chrono.ChVisualShapeFEA(mesh)                           # colored bending-moment surface
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # bending moment Mz field
vis_beam.SetColorscaleMinMax(-0.4, 0.4)                            # color scale limits
vis_beam.SetSmoothFaces(True)                                      # smooth shading
vis_beam.SetWireframe(False)                                       # solid surface
mesh.AddVisualShapeFEA(vis_beam)                                   # register the surface shape

vis_nodes = chrono.ChVisualShapeFEA(mesh)                          # node coordinate-system glyphs
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)  # draw node triads
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)    # no scalar field on glyphs
vis_nodes.SetSymbolsThickness(0.006)                              # glyph line thickness
vis_nodes.SetSymbolsScale(0.01)                                   # glyph size
vis_nodes.SetZbufferHide(False)                                   # always draw glyphs
mesh.AddVisualShapeFEA(vis_nodes)                                 # register the glyph shape

sys.SetSolver(mkl.ChSolverPardisoMKL())                            # direct solver for stiff beams

ts = chrono.ChTimestepperHHT(sys)                                  # HHT implicit timestepper
ts.SetStepControl(False)                                          # canonical-minimal HHT setup
sys.SetTimestepper(ts)                                            # install the timestepper

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht render window
vis.AttachSystem(sys)                                             # bind the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)                 # Y-up camera convention
vis.SetWindowSize(1280, 720)                                      # window resolution
vis.SetWindowTitle("Beam Buckling (IGA FEA)")                     # window title
vis.Initialize()                                                 # create device BEFORE scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                  # sky box
vis.AddCamera(chrono.ChVector3d(1.0, 0.6, 1.4),                   # camera eye
              chrono.ChVector3d(0, 0.5, 0))                       # look at the column
vis.AddTypicalLights()                                           # standard lighting

time_step = 1e-3                                                  # stiff-beam timestep
sim_end = 4.0                                                     # total simulated time
render_fps = 50.0                                                 # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))     # physics steps per rendered frame
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
