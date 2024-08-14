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

# Set automatic gravity for the mesh
mesh.SetAutomaticGravity(True, 2)  # for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA
sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

# Beam parameters
beam_L = 6
beam_ro = 0.050
beam_ri = 0.045
CH_PI = 3.1416  # Corrected value of PI

# Create a section, i.e. thickness and material properties for beams
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoissonRatio(0.3)
melasticity.SetIyy((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((CH_PI / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((CH_PI / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)  # SetAsCircularSection(..) would overwrite Ixx Iyy J etc.

# Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements
builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,  # the mesh to put the elements in
                  msection,  # section of the beam
                  20,  # number of sections (spans)
                  chrono.ChVector3d(0, 0, 0),  # start point
                  chrono.ChVector3d(beam_L, 0, 0),  # end point
                  chrono.VECT_Y,  # suggested Y direction of section
                  3)  # order (3 = cubic, etc)

node_mid = builder.GetLastBeamNodes()[m.floor(builder.GetLastBeamNodes().size() / 2.0)]

# Create the flywheel and attach it to the center of the beam
mbodyflywheel = chrono.ChBodyEasyCylinder(0.24, 0.1, 7800)  # R, h, density
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),  # flywheel initial center (plus Y offset)
                       chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Z))
    # flywheel initial alignment (rotate 90° so cylinder axis is on X)
)
sys.Add(mbodyflywheel)

myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)

# Create the truss
truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)

# Create the end bearing
bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(),
                   truss,
                   chrono.ChFrameD(builder.GetLastBeamNodes().back().GetPos())
                   )
sys.Add(bearing)

# Create the motor that rotates the beam
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

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
            return self.A1 * x
        elif x < self.T2:
            return self.A1 * self.T1
        elif x < self.T3:
            return self.A2 * m.sin(self.w * (x - self.T2))
        else:
            return 0

# Instantiate the custom motor function
custom_motor_function = ChFunctionMyFun(A1=0.1, A2=0.2, T1=1, T2=2, T3=3, w=2*m.pi)

# Connect the rotor and the stator and add the motor to the system
rotmotor1.Initialize(builder.GetLastBeamNodes().front(),  # body A (slave)
                     truss,  # body B (master)
                     chrono.ChFrameD(builder.GetLastBeamNodes().front().GetPos(),
                                     chrono.Q_from_AngAxis(CH_PI / 2.0, chrono.VECT_Y))
                     # motor frame, in abs. coords
                     )
rotmotor1.SetMotorFunction(custom_motor_function)
sys.Add(rotmotor1)

# Attach a visualization of the FEM mesh
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

# Set up the solver
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

# Optionally, set up a more precise HHT timestepper
# timestepper = chrono.ChTimestepperHHT(sys)
# timestepper.SetAlpha(-0.2)
# timestepper.SetMaxiters(100)
# timestepper.SetAbsTolerances(1e-10)
# timestepper.SetMode(chrono.ChTimestepperHHT.POSITION)
# timestepper.SetScaling(True)
# sys.SetTimestepper(timestepper)

sys.DoStaticLinear()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)