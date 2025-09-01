import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Create the system
sys = chrono.ChSystemSMC()

# Create the mesh
mesh = fea.ChMesh()
sys.Add(mesh)

# Set mesh properties
mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Beam parameters
beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.14159  # Corrected value of pi

# Create beam section properties
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build the beam
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(beam_L, 0, 0), chrono.VECT_Y, 3)

# Get middle node
nodes = builder.GetLastBeamNodes()
node_mid = nodes[int(m.floor(nodes.size() / 2.0))]

# Create flywheel
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z))
)
sys.Add(mbodyflywheel)

# Connect flywheel to beam
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create end bearing (fixed translations, free rotation about X)
bearing = chrono.ChLinkMateGeneric(True, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Create motor
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),
                     truss,
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y)))
sys.Add(rotmotor1)

# Custom motor function class
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self, A1, A2, T1, T2, T3, w):
        chrono.ChFunction.__init__(self)
        self.A1 = A1
        self.A2 = A2
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.w = w

    def GetVal(self, x):
        if x < self.T1:
            return self.A1 * m.sin(self.w * x)
        elif x < self.T2:
            return self.A2
        elif x < self.T3:
            return self.A2 - (self.A2 - self.A1) * m.sin(self.w * (x - self.T2))
        else:
            return self.A1

# Set custom motor function
custom_motor = ChFunctionMyFun(A1=10, A2=50, T1=1, T2=3, T3=5, w=0.5)
rotmotor1.SetMotorFunction(custom_motor)

# Visualization for FEM mesh
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

# Create Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: the Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Set solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Set HHT timestepper for better accuracy
ts = chrono.ChTimestepperHHT(sys)
ts.SetAlpha(-0.1)  # Numerical damping parameter
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)