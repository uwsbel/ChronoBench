import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045

# Create section with corrected math.pi usage
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(m.pi * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((m.pi / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build beam with corrected node access
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, 
                  chrono.ChVector3d(0, 0, 0), 
                  chrono.ChVector3d(beam_L, 0, 0), 
                  chrono.VECT_Y, 1)

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]

# Create flywheel with corrected quaternion
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(m.pi/2, chrono.VECT_Z))
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(), truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Custom motor function implementation
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.A1 = 10.0   # Initial speed (rad/s)
        self.A2 = 40.0   # Max speed (rad/s)
        self.T1 = 1.0    # Ramp start time
        self.T2 = 3.0    # Ramp end time
        self.T3 = 5.0    # Sine phase end time
        self.w = 0.4*m.pi  # Angular frequency (0.4π rad/s)

    def GetVal(self, x):
        if x < self.T1:
            return self.A1
        elif x < self.T2:
            return self.A1 + (self.A2 - self.A1) * (x - self.T1)/(self.T2 - self.T1)
        elif x < self.T3:
            return self.A2 * m.sin(self.w * x)
        else:
            return 0.0

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(), truss,
                    chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                    chrono.QuatFromAngleAxis(m.pi/2, chrono.VECT_Y)))
sys.Add(rotmotor1)

f_custom = ChFunctionMyFun()
rotmotor1.SetMotorFunction(f_custom)

# Visualization (retained from original)
mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Irrlicht setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified Jeffcott Rotor')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L/2, 0, 0))
vis.AddTypicalLights()

# Solver and timestepping
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Optional HHT timestepper (uncomment to use)
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
# hht_stepper = sys.GetTimestepper().AsTimestepperHHT()
# hht_stepper.SetAlpha(-0.2)
# hht_stepper.SetStepControl(True)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)