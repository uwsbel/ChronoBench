import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

# Custom motor function class
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        # Define parameters for the complex motor function
        self.A1 = 2.0      # Amplitude 1
        self.A2 = 1.5      # Amplitude 2  
        self.T1 = 1.0      # Time threshold 1
        self.T2 = 3.0      # Time threshold 2
        self.T3 = 5.0      # Time threshold 3
        self.w = 10.0      # Angular frequency
    
    def GetVal(self, x):
        # Piecewise function based on time x
        if x < self.T1:
            # Linear ramp up
            return self.A1 * x / self.T1
        elif x < self.T2:
            # Sinusoidal variation
            return self.A1 + self.A2 * m.sin(self.w * (x - self.T1))
        elif x < self.T3:
            # Exponential decay with oscillation
            decay_factor = m.exp(-(x - self.T2))
            return self.A1 * decay_factor * m.cos(self.w * (x - self.T2))
        else:
            # Constant value
            return 0.5 * self.A1

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True,
                         2)  # for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
# Fixed: Correct value of PI
CH_PI = m.pi  # Use math.pi for accuracy

# Create a section, i.e. thickness and material properties
# for beams. This will be shared among some beams.

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
msection.SetDrawCircularRadius(beam_ro)  # SetAsCircularSection(..) would overwrite Ixx Iyy J etc.

# Use the ChBuilderBeamIGA tool for creating a straight rod
# divided in Nel elements:

builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,  # the mesh to put the elements in
                  msection,  # section of the beam
                  20,  # number of sections (spans)
                  chrono.ChVector3d(0, 0, 0),  # start point
                  chrono.ChVector3d(beam_L, 0, 0),  # end point
                  chrono.VECT_Y,  # suggested Y direction of section
                  1)  # order (3 = cubic, etc)

# Fixed: Proper integer conversion for array indexing
node_mid = builder.GetLastBeamNodes()[int(m.floor(builder.GetLastBeamNodes().size() / 2.0))]

# Create the flywheel and attach it to the center of the beam

mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)  # R, h, density
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  # flywheel initial center (plus Y offset)
                       chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Z))
    # flywheel initial alignment (rotate 90° so cylinder axis is on X)
)
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
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos())
                   )
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  # body A (slave)
                     truss,  # body B (master)
                     chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.QuatFromAngleAxis(CH_PI / 2.0, chrono.VECT_Y))
                     # motor frame, in abs. coords
                     )
sys.Add(rotmotor1)

# Modified: Replace simple sine function with custom complex motor function
f_complex = ChFunctionMyFun()
rotmotor1.SetMotorFunction(f_complex)

# Attach a visualization of the FEM mesh.

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

msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Optional: More precise HHT timestepper (commented out for now)
# stepper = chrono.ChTimestepperHHT(sys)
# stepper.SetAlpha(-0.2)
# stepper.SetMaxIters(100)
# stepper.SetAbsTolerances(1e-5)
# sys.SetTimestepper(stepper)

sys.DoStaticLinear()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)