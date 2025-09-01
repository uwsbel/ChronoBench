import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

# --- Modified Gear Sizes ---
radA = 1.5                                        # Changed radius for first gear
radB = 3.5                                        # Changed radius for second gear

# Create the truss
sys_dimen_x = 15
sys_dimen_y = 8
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2,     # Original: 20x10x2
                                   1000,          # Setting mass (not used for fixed body)
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
# --- Modified Truss Dimensions ---
# Updated to match new specified dimensions
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,  # Modified dimensions
                                   1000,
                                   True,
                                   False,
                                   mat)
sys.Add(mbody_truss)
mbody_truss.SetFixed(True)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,
                                   1000,
                                   True,
                                   False,
                                   mat)
sys.Add(mbody_train)
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))

# Create a revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(
    mbody_truss,
    mbody_train,
    chrono.ChFramef(chrono.ChVector3d(0, 0, 0), chrono.Q_FROM_IDENTITY)
)
sys.AddLink(link_revoluteTT)

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(
    radA,
    0.5,
    1000,
    True,
    False,
    mat
)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
# --- Rotating gear by 90 degrees around X-axis ---
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Creating visualization shaft for gear A with adjusted size
# --- Size changed to radius * 0.3, length 10 ---
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)  # Modified size
mbody_gearA.AddVisualShape(
    mshaft_shape,
    chrono.ChFramef(
        chrono.ChVector3f(0, 3.5, 0),
        chrono.QuatFromAngleX(chrono.CH_PI_2)
    )
)

# Impose rotation speed on gear A
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(
    mbody_gearA,
    mbody_truss,
    chrono.ChFramef(chrono.ChVector3f(0, 0, 0), chrono.Q_UNIT)
)
# --- Changed rotation speed to 3 ---
link_motor.SetSpeedFunction(chrono.ChFunctionConstant(3))
sys.AddLink(link_motor)

# Create the second gear
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
# --- Changed position of gear B ---
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
# Keep the same rotation
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix second gear to rotating bar with revolute joint
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(
    mbody_gearB,
    mbody_train,
    chrono.ChFramef(chrono.ChVector3d(interaxis12, 0, 0), chrono.Q_UNIT)
)
sys.AddLink(link_revolute)

# Create gear constraint between gears A and B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(
    mbody_gearA,
    mbody_gearB,
    chrono.ChFramef()
)
# --- Set frames for shafts with proper orientation ---
link_gearAB.SetFrameShaft1(chrono.ChFramef(
    chrono.ChVector3f(0, 0, 0),
    chrono.QuatFromAngleX(-m.pi / 2)
))
link_gearAB.SetFrameShaft2(chrono.ChFramef(
    chrono.ChVector3f(0, 0, 0),
    chrono.QuatFromAngleX(-m.pi / 2)
))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Create gear constraint between second gear B and large wheel C (which is the truss)
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(
    mbody_gearB,
    mbody_truss,
    chrono.ChFramef()
)
# --- Set frames with proper orientation ---
link_gearBC.SetFrameShaft1(chrono.ChFramef(
    chrono.ChVector3f(0, 0, 0),
    chrono.QuatFromAngleX(-m.pi / 2)
))
# --- Changed size of shaft to radius * 0.3 and length 10 as per instruction ---
link_gearBC.SetFrameShaft2(chrono.ChFramef(
    chrono.ChVector3f(0, 0, -4),
    chrono.QuatFromAngleX(-m.pi / 2)
))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# --- Visualization ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)