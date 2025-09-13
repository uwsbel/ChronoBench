import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Custom motor function class with piecewise-defined rotational speed
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        # Parameters for the piecewise function
        self.A1 = 20.0   # Initial ramp rate (rad/s^2)
        self.A2 = -10.0  # Secondary ramp rate (rad/s^2)
        self.T1 = 1.0    # First time threshold (s)
        self.T2 = 2.0    # Second time threshold (s)
        self.T3 = 4.0    # Third time threshold (s)
        self.w = 40.0    # Target angular velocity (rad/s)
    
    def GetVal(self, x):
        """Compute angular speed based on piecewise conditions"""
        if x < self.T1:
            return self.A1 * x  # Ramp up
        elif x < self.T2:
            return self.A1 * self.T1 + self.A2 * (x - self.T1)  # Ramp down
        elif x < self.T3:
            return self.A1 * self.T1 + self.A2 * (self.T2 - self.T1) + self.w * (x - self.T2)  # Constant acceleration
        else:
            return self.A1 * self.T1 + self.A2 * (self.T2 - self.T1) + self.w * (self.T3 - self.T2)  # Constant speed

# Create system
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)

# Enable automatic gravity with improved precision
mesh.SetAutomaticGravity(True)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Beam parameters
beam_L = 6
beam_ro = 0.050
beam_ri = 0.045

# Create beam section with material properties
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(math.pi * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((math.pi / 4) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((math.pi / 4) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((math.pi / 4) * (beam_ro**4 - beam_ri**4))
melasticity.SetIzz((math.pi / 4) * (beam_ro**4 - beam_ri**4))
melasticity.SetJ((math.pi / 2) * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build beam with IGA elements
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh, 
    msection, 
    20,  # Number of elements
    chrono.ChVector3d(0, 0, 0), 
    chrono.ChVector3d(beam_L, 0, 0), 
    chrono.VECT_Y, 
    3    # Cubic order
)

# Get middle node (using integer division)
node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]

# Create and attach flywheel
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(math.pi / 2, chrono.VECT_Z)
    )
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create fixed truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create bearing constraint (corrected frame initialization)
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    builder.GetLastBeamNodes().back(),
    truss,
    chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos())
)
sys.Add(bearing)

# Create rotation motor with custom function
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes().front(),
    truss,
    chrono.ChFrameD(
        builder.GetLastBeamNodes().front().GetPos(),
        chrono.QuatFromAngleAxis(math.pi / 2, chrono.VECT_Y)
    )
)
sys.Add(rotmotor1)

# Use custom motor function instead of sine wave
rotmotor1.SetMotorFunction(ChFunctionMyFun())

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

# Irrlicht setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified FEA: Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Solver and timestepping
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Enable HHT timestepper for improved accuracy (optional)
sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)