import math
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Define custom motor function class
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)
        self.A1 = 40.0   # Amplitude 1 (rad/s)
        self.A2 = 0.2    # Amplitude 2 (rad/s)
        self.T1 = 0.5    # Time threshold 1 (s)
        self.T2 = 1.0    # Time threshold 2 (s)
        self.T3 = 1.5    # Time threshold 3 (s)
        self.w = 40.0    # Angular frequency (rad/s)
    
    def GetVal(self, x):
        """Piecewise motor speed function based on time x"""
        if x < self.T1:
            return 0.0  # Stationary before T1
        elif x < self.T2:
            # Linear ramp from 0 to A1
            return self.A1 * (x - self.T1) / (self.T2 - self.T1)
        elif x < self.T3:
            return self.A1  # Constant speed
        else:
            # Constant + sinusoidal component
            return self.A1 + self.A2 * math.sin(self.w * (x - self.T3))
    
    def Clone(self):
        """Create a copy of the function"""
        new_func = ChFunctionMyFun()
        new_func.A1 = self.A1
        new_func.A2 = self.A2
        new_func.T1 = self.T1
        new_func.T2 = self.T2
        new_func.T3 = self.T3
        new_func.w = self.w
        return new_func

# Create physical system
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)

# Configure gravity
mesh.SetAutomaticGravity(True, 2)  # 2 integration points for accuracy
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Beam parameters
beam_L = 6
beam_ro = 0.050
beam_ri = 0.045

# Create beam section properties
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(math.pi * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((math.pi / 4) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((math.pi / 4) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy(minertia.GetIyy())
melasticity.SetIzz(minertia.GetIzz())
melasticity.SetJ(math.pi/2 * (beam_ro**4 - beam_ri**4))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build beam with IGA
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(
    mesh,                   # Mesh to populate
    msection,               # Beam section properties
    20,                     # Number of elements
    chrono.ChVector3d(0, 0, 0),  # Start point
    chrono.ChVector3d(beam_L, 0, 0),  # End point
    chrono.ChVector3d(0, 1, 0),  # Section direction
    3                       # Cubic order (for smooth IGA)
)

# Get middle node
nodes = builder.GetLastBeamNodes()
node_mid = nodes[len(nodes) // 2]

# Create flywheel and attach to beam center
mbodyflywheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Y, 0.24, 0.1, 7800  # Radius, height, density
)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  # Position
        chrono.QuatFromAngleAxis(math.pi / 2, chrono.ChVector3d(0, 0, 1))  # Orientation
    )
)
sys.Add(mbodyflywheel)

# Fix flywheel to beam
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create fixed truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Create bearing constraint at end node (allows rotation about X)
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(
    builder.GetLastBeamNodes().back(),  # End node
    truss,                              # Fixed truss
    chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos())  # Position
)
sys.Add(bearing)

# Create rotation motor at start node
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(
    builder.GetLastBeamNodes().front(),  # First node
    truss,                              # Fixed truss
    chrono.ChFrameD(  # Motor frame (aligned with beam axis)
        builder.GetLastBeamNodes().front().GetPos(),
        chrono.QUNIT  # Identity rotation (X along beam)
    )
)
sys.Add(rotmotor1)

# Set custom motor function
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

# Optional: Enable more precise HHT integrator (uncomment to use)
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
# hht_ts = sys.GetTimestepper().AsTimestepperHHT()
# hht_ts.SetAlpha(-0.2)
# hht_ts.SetStepControl(True)

# Setup Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified FEA: Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Setup solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Perform static analysis (pre-stress)
sys.DoStaticLinear()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)