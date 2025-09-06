import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        super().__init__()
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def Get_val(self, x):
        if x < self.T1:
            return self.A1 * m.sin(self.w * x)
        elif self.T1 <= x < self.T2:
            return self.A2 * m.sin(self.w * x)
        elif self.T2 <= x < self.T3:
            return self.A1 * m.sin(self.w * x)
        else:
            return 0.0

sys = chrono.ChSystemSMC()

# Optional: Set HHT timestepper for better stability
# Uncomment the following line if needed
# sys.SetTimestepper(chrono.ChTimestepperHHT(sys))

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))  # Corrected to ChVectorD

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = m.pi  # Use math.pi for better precision

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetJ((CH_PI / 2.0) * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                  msection,
                  20,
                  chrono.ChVectorD(0, 0, 0),
                  chrono.ChVectorD(beam_L, 0, 0),
                  chrono.VECT_Y,
                  1)

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.VECT_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0))
mbodyflywheel.SetRot(chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1], truss, chrono.ChFrameD(builder.GetLastBeamNodes()[-1].GetPos()))
sys.Add(bearing)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
front_node = builder.GetLastBeamNodes()[0]
rotmotor1.Initialize(front_node, truss, chrono.ChFrameD(front_node.GetPos(), chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y)))

# Custom motor function parameters
A1 = 0.5
A2 = 1.0
T1 = 1.0
T2 = 2.0
T3 = 3.0
w = 2 * m.pi
my_func = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(my_func)
sys.Add(rotmotor1)

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

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 4), chrono.ChVectorD(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisomkl()
sys.SetSolver(msolver)

sys.DoStaticLinear()  # For initial static equilibrium

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

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

sys = chrono.ChSystemSMC()

# Optional: Set HHT timestepper for better stability
# Uncomment the following line if needed
# sys.SetTimestepper(chrono.ChTimestepperHHT(sys))

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = m.pi  # Use math.pi for better precision

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetJ((CH_PI / 2.0) * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                  msection,
                  20,
                  chrono.ChVectorD(0, 0, 0),
                  chrono.ChVectorD(beam_L, 0, 0),
                  chrono.VECT_Y,
                  1)

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.VECT_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0))
mbodyflywheel.SetRot(chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1], truss, chrono.ChFrameD(builder.GetLastBeamNodes()[-1].GetPos()))
sys.Add(bearing)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
front_node = builder.GetLastBeamNodes()[0]
rotmotor1.Initialize(front_node, truss, chrono.ChFrameD(front_node.GetPos(), chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y)))

# Custom motor function parameters
A1 = 0.5
A2 = 1.0
T1 = 1.0
T2 = 2.0
T3 = 3.0
w = 2 * m.pi
my_func = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(my_func)
sys.Add(rotmotor1)

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

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 4), chrono.ChVectorD(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisomkl()
sys.SetSolver(msolver)

sys.DoStaticLinear()  # For initial static equilibrium

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

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

sys = chrono.ChSystemSMC()

# Optional HHT timestepper for improved stability
# sys.SetTimestepper(chrono.ChTimestepperHHT(sys))

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = m.pi  # Use precise math.pi value

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetIzz((CH_PI / 4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetJ((CH_PI / 2.0) * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                 msection,
                 20,
                 chrono.ChVectorD(0, 0, 0),
                 chrono.ChVectorD(beam_L, 0, 0),
                 chrono.VECT_Y,
                 1)

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.VECT_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0))
mbodyflywheel.SetRot(chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes()[-1], truss, chrono.ChFrameD(builder.GetLastBeamNodes()[-1].GetPos()))
sys.Add(bearing)

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
front_node = builder.GetLastBeamNodes()[0]
rotmotor1.Initialize(front_node, truss, chrono.ChFrameD(front_node.GetPos(), chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y)))

# Custom motor function parameters
A1 = 0.5
A2 = 1.0
T1 = 1.0
T2 = 2.0
T3 = 3.0
w = 2 * m.pi
my_func = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(my_func)
sys.Add(rotmotor1)

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

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: Jeffcott Rotor with IGA Beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 4), chrono.ChVectorD(beam_L / 2, 0, 0))
vis.AddTypicalLights()

msolver = mkl.ChSolverPardisomkl()
sys.SetSolver(msolver)

sys.DoStaticLinear()  # Initial static equilibrium

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)