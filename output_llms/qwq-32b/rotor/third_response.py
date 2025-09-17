import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()  # Changed to NSC for HHT timestepper

# Custom motor function class
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        super().__init__()
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def Get_y(self, x):
        if x < self.T1:
            return self.A1 * m.sin(self.w * x)
        elif self.T1 <= x < self.T2:
            return self.A2 * m.sin(self.w * x)
        elif self.T2 <= x < self.T3:
            return self.A1 * m.sin(self.w * x)
        else:
            return 0.0

# Set up HHT timestepper
timestepper = chrono.ChTimestepperHHT(sys)
sys.SetTimestepper(timestepper)

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045

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

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    20,
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(beam_L, 0, 0),
    chrono.VECT_Y,
    3  # Changed order to 3 for cubic IGA
)

node_mid = builder.GetLastBeamNodes()[int(len(builder.GetLastBeamNodes())/2)]

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordSys(
    chrono.ChCoordSysD(
        node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0),
        chrono.Q_from_AngAxis(m.pi/2.0, chrono.VECT_Z)
    )
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    builder.GetLastBeamNodes()[-1],  # Fixed end node
    truss,
    chrono.ChFrameD(builder.GetLastBeamNodes()[-1].GetPos())
)
sys.Add(bearing)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes()[0],  # Front node
    truss,
    chrono.ChFrameD(
        builder.GetLastBeamNodes()[0].GetPos(),
        chrono.Q_from_AngAxis(m.pi/2.0, chrono.VECT_Y)
    )
)
sys.Add(rotmotor1)

# Custom motor function parameters
A1 = 2.0
A2 = 4.0
T1 = 1.0
T2 = 2.0
T3 = 3.0
w = 2 * m.pi * 10  # 10 Hz frequency
my_fun = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(my_fun)

# Visualization setup
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
vis.SetWindowTitle('Test FEA: Jeffcott Rotor with IGA Beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 4), chrono.ChVectorD(beam_L/2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisomkl()
sys.SetSolver(msolver)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)