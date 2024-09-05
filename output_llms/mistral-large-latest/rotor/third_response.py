import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Define the custom motor function class
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
            return self.A1 * (1 - m.cos(self.w * x))
        elif x < self.T2:
            return self.A1
        elif x < self.T3:
            return self.A1 + self.A2 * m.sin(self.w * (x - self.T2))
        else:
            return self.A1

# Initialize the Chrono system
sys = chrono.ChSystemSMC()

# Create and add the FEA mesh
mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)  # for max precision in gravity of FE
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# Beam dimensions and properties
beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = m.pi

# Create a section for the beam
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (m.pow(beam_ro, 2) - m.pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (m.pow(beam_ro, 4) - m.pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (m.pow(beam_ro, 4) - m.pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulus(0.3)  # Poisson's ratio
melasticity.SetIyy((CH_PI / 4.0) * (m.pow(beam_ro, 4) - m.pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (m.pow(beam_ro, 4) - m.pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (m.pow(beam_ro, 4) - m.pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build the beam using IGA tool
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(beam_L, 0, 0), chrono.VECT_Y, 3)

node_mid = builder.GetLastBeamNodes()[m.floor(len(builder.GetLastBeamNodes()) / 2.0)]

# Create the flywheel and attach it to the center of the beam
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordsys(chrono.ChCoordsysD(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0),
                                             chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Z)))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1], truss, chrono.ChCoordsysD(builder.GetLastBeamNodes()[-1].GetPos()))
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes()[0], truss, chrono.ChFrame(builder.GetLastBeamNodes()[0].GetPos(),
                                                                         chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

# Custom motor function
f_ramp = ChFunctionMyFun(A1=10, A2=5, T1=2, T2=4, T3=6, w=2 * CH_PI)
rotmotor1.SetMotorFunction(f_ramp)

# Attach visualization of the FEM mesh
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
vis.AddCamera(chrono.ChVectorD(0, 1, 4), chrono.ChVectorD(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Use Pardiso MKL solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Optionally enable HHT timestepper
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

sys.DoStaticLinear()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)