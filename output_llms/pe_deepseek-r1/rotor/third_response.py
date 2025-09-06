import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Custom motor function implementation
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        # Motor parameters
        self.A1 = 40.0  # Initial amplitude (rad/s)
        self.A2 = 80.0  # Final amplitude (rad/s)
        self.T1 = 0.5   # Time segment 1 end (s)
        self.T2 = 1.0   # Time segment 2 end (s)
        self.T3 = 1.5   # Time segment 3 end (s)
        self.w = 10.0   # Angular frequency for oscillation (rad/s)
    
    def GetVal(self, x):
        """Piecewise motor speed function based on time"""
        if x < self.T1:
            return self.A1 * x / self.T1  # Linear ramp up
        elif x < self.T2:
            return self.A1  # Constant speed
        elif x < self.T3:
            # Linear transition between A1 and A2
            return self.A1 + (self.A2 - self.A1) * (x - self.T2) / (self.T3 - self.T2)
        else:
            # Oscillating speed after T3
            return self.A2 * m.sin(self.w * x)

# System setup
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045

# Create beam section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(chrono.CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((chrono.CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((chrono.CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)

# Build beam
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20, 
                  chrono.ChVector3d(0, 0, 0), 
                  chrono.ChVector3d(beam_L, 0, 0), 
                  chrono.VECT_Y, 1)

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]

# Create flywheel with corrected orientation
mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetPos(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0))
# Corrected rotation: First rotate 90° about Z, then 90° about Y
mbodyflywheel.SetRot(chrono.QuatFromAngleAxis(chrono.CH_PI / 2, chrono.VECT_Z) * 
                     chrono.QuatFromAngleAxis(chrono.CH_PI / 2, chrono.VECT_Y))
sys.Add(mbodyflywheel)

# Connect flywheel to beam
myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create truss
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Corrected bearing with 7 parameters (all translations fixed, only X rotation free)
bearing = chrono.ChLinkMateGeneric(True, True, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(), truss, 
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
sys.Add(bearing)

# Create motor with custom function
rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(), truss,
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(chrono.CH_PI / 2, chrono.VECT_Y)))
sys.Add(rotmotor1)

# Use custom motor function
custom_motor_fun = ChFunctionMyFun()
rotmotor1.SetMotorFunction(custom_motor_fun)

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

# Visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Modified FEA: Jeffcott rotor with custom motor')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()

# Solver and timestepper
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Optional HHT timestepper for higher precision (uncomment to use)
# sys.SetTimestepperType(chrono.ChTimestepper.Type_HHT)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)