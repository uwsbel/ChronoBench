import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import math

class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        super().__init__()
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):
        if x < self.T1:
            return self.A1 * math.sin(self.w * x)
        elif x < self.T2:
            return self.A1 + (self.A2 - self.A1) * (x - self.T1) / (self.T2 - self.T1)
        elif x < self.T3:
            return self.A2 * math.sin(self.w * (x - self.T2 + self.T1))
        else:
            return 0

    def Clone(self):
        return ChFunctionMyFun(self.A1, self.A2, self.T1, self.T2, self.T3, self.w)

# Create the system and mesh
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)
mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Define beam properties
beam_L = 6
beam_ro = 0.050
beam_ri = 0.045

# Create a section for the beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(chrono.CH_C_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((chrono.CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((chrono.CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((chrono.CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((chrono.CH_C_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((chrono.CH_C_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build the beam
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(beam_L, 0, 0), chrono.VECT_Y, 1)

node_mid = builder.GetLastBeamNodes()[math.floor(len(builder.GetLastBeamNodes()) / 2.0)]

# Create and add the flywheel
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordsys(chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0), chrono.QuatFromAngleAxis(chrono.CH_C_PI / 2.0, chrono.VECT_Z)))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss and bearing
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(), truss, chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Create and add the custom motor
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(), truss, chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(), chrono.QuatFromAngleAxis(chrono.CH_C_PI / 2.0, chrono.VECT_Y)))

# Custom motor function
A1, A2, T1, T2, T3, w = 40, 20, 0.5, 1.5, 3, 0.2
f_ramp = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(f_ramp)
sys.Add(rotmotor1)

# Visualization settings
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

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Solver and time step
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)  # Uncomment for HHT timestepper

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)

