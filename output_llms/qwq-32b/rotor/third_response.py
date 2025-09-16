import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

# Removed invalid mesh.SetAutomaticGravity call
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = chrono.CH_C_PI  # Use Chrono's constant for better precision

minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,
    msection,
    20,  # number of elements (spans)
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(beam_L, 0, 0),
    chrono.VECT_Y,
    3  # Changed to cubic IGA (order 3)
)

node_mid_index = len(builder.GetLastBeamNodes()) // 2
node_mid = builder.GetLastBeamNodes()[node_mid_index]

# Create the flywheel and attach it to the center of the beam
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVectorD(0, 0.05, 0))
mbodyflywheel.SetRot(chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Z))
sys.Add(mbodyflywheel)

# Use ChLinkLockPointFrame to constrain the flywheel to the node's position
flywheel_link = chrono.ChLinkLockPointFrame()
flywheel_link.Initialize(mbodyflywheel, chrono.ChFrameD(node_mid.GetPos()))
sys.Add(flywheel_link)

# Create the truss and end bearing
truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)

bearing_node = builder.GetLastBeamNodes()[-1]
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(bearing_node, truss, chrono.ChFrameD(bearing_node.GetPos()))
sys.Add(bearing)

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
        elif x < self.T2:
            return self.A2 * m.cos(self.w * (x - self.T1))
        elif x < self.T3:
            return (self.A1 + self.A2) * m.sin(self.w * (x - self.T2))
        else:
            return 0.0

# Create motor with custom function
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes()[0],  # body A (slave)
    truss,  # body B (master)
    chrono.ChFrameD(builder.GetLastBeamNodes()[0].GetPos(),
                    chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y))
)
sys.Add(rotmotor1)

# Define parameters for the custom function
A1 = 1.0  # Example parameter values
A2 = 2.0
T1 = 1.0
T2 = 2.0
T3 = 3.0
w = 2 * m.pi * 1.0  # 1 Hz
custom_fun = ChFunctionMyFun(A1, A2, T1, T2, T3, w)
rotmotor1.SetMotorFunction(custom_fun)

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

# Optional HHT timestepper setup (uncomment to use)
# hht = chrono.ChTimestepperHHT(sys)
# hht.SetAlpha(-0.2)
# hht.SetMaxiters(3)
# hht.SetAbsTolerances(1e-5)
# sys.SetTimestepper(hht)

# Initialize simulation
sys.DoStaticLinear()

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

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)