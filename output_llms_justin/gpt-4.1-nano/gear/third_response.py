import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

# Create all rigid bodies with specific dimensions
radA = 1.5                                        # Radius for gear A
radB = 3.5                                        # Radius for gear B

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      # Modified box-shaped truss body with dimensions 15x8x2
                                   1000,          # Setting mass (not used for fixed body)
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_truss)                              # Adding the truss to the system
mbody_truss.SetFixed(True)                        # Making the truss fixed (immovable)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))    # Setting the position of the truss to (0, 0, 3)

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()                       # Creating a visual material
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  # Setting a texture for the visual material

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  # Creating a box-shaped rotating bar with dimensions 8x1.5x1.0
                                   1000,          # Setting mass
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_train)                              # Adding the rotating bar to the system
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))    # Positioning the rotating bar at (3, 0, 0)

# Create a revolute joint between truss and rotating bar, allowing rotation along the Z-axis
link_revoluteTT = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  # Initializing the joint with truss and rotating bar
                           chrono.ChFrameD(chrono.ChVector3d(0,0,3),  # Position at truss top
                                           chrono.Q_from_AngAxis(0, chrono.ChVector3D(0,0,1))))             # No rotation
sys.AddLink(link_revoluteTT)                                          # Adding the joint to the system

# Create the first gear (gear A)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             # Creating a cylindrical gear with Y axis as the central axis
                                        radA, 0.5,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearA)                                                # Adding the gear to the system
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add a visual shape for gear A (optional, for better visualization)
gearA_vis_shape = chrono.ChVisualShapeCylinder(radA, 0.5)
mbody_gearA.AddVisualShape(gearA_vis_shape, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), chrono.Quat(1,0,0,0)))
# Rotate for better visualization if needed

# Impose rotation speed on gear A
link_motor = chrono.ChLinkMotorRotationSpeed()
link_motor.Initialize(mbody_gearA, mbody_truss, chrono.ChFrameD(chrono.ChVector3d(0, 0, -1), chrono.Q_from_AngX(0)))
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))
sys.AddLink(link_motor)

# Create gear B
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radB, 0.4, 1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Fix gear B to rotating bar with a revolute joint
link_revolute = chrono.ChLinkLockRevolute()
link_revolute.Initialize(mbody_gearB, mbody_train,
                         chrono.ChFrameD(chrono.ChVector3d(interaxis12, 0, -1), chrono.Q_from_AngX(0)))
sys.AddLink(link_revolute)

# Setting the gear ratio between gear A and B (1:1)
# Add gear constraint
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB,
                       chrono.ChFrameD(chrono.ChVector3d(0, 0, -1), chrono.Q_from_AngX(0)))
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.ChVector3d(0,0, -1), chrono.Q_from_AngX(0)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0,0, -1), chrono.Q_from_AngX(0)))
link_gearAB.SetTransmissionRatio(1.0)  # 1:1 ratio
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Create the large wheel C attached internally to gear B for illustration (optional)
radC = 2 * radB + radA
# For simplicity, we consider it fixed, or ignore visual shape if not needed

# ------------------ New Bevel Gear (gear D) ------------------
radD = 5.0
# Create gear D
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radD, 0.5, 1000, True, False, mat)
sys.Add(mbody_gearD)
# Position at (-10, 0, -9)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
# Rotate gear D by 90 degrees around Z-axis
qDrotation = chrono.Q_from_AngleAxis(m.pi / 2, chrono.ChVector3D(0, 0, 1))
mbody_gearD.SetRot(qDrotation)
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add visual shape for gear D
gearD_vis_shape = chrono.ChVisualShapeCylinder(radD, 0.5)
mbody_gearD.AddVisualShape(gearD_vis_shape, chrono.ChFrameD(chrono.ChVector3d(0,0,0), qDrotation))

# Revolute joint to connect gear D to the truss along a horizontal axis (Z axis)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_truss, mbody_gearD,
                          chrono.ChFrameD(chrono.ChVector3d(-10, 0, -9), chrono.Q_from_AngAxis(0, chrono.ChVector3D(0,0,1))))
sys.Add(link_revoluteD)

# Gear ratio between gear A and D (1:1)
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD,
                       chrono.ChFrameD(chrono.ChVector3d(0,0,-1), chrono.Q_from_AngX(0)))
link_gearAD.SetFrameShaft1(chrono.ChFrameD(chrono.ChVector3d(0,0,-1), chrono.Q_from_AngX(0)))
link_gearAD.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0,0,-1), chrono.Q_from_AngX(0)))
link_gearAD.SetTransmissionRatio(1.0)
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)

# ------------------ New Pulley E ------------------
pulley_radius = 2.0
# Create pulley E
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, pulley_radius, 0.3, 1000, True, False, mat)
sys.Add(mbody_pulleyE)
# Position at (-10, -11, -9)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
# Rotate by 90 degrees around Z (optional visual)
qErotation = chrono.Q_from_AngleAxis(m.pi / 2, chrono.ChVector3D(0, 0, 1))
mbody_pulleyE.SetRot(qErotation)
# Add visual shape
pulley_vis_shape = chrono.ChVisualShapeCylinder(pulley_radius, 0.3)
mbody_pulleyE.AddVisualShape(pulley_vis_shape, chrono.ChFrameD(chrono.ChVector3d(0,0,0), qErotation))

# Revolute joint to connect pulley E to the truss along horizontal axis
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_truss, mbody_pulleyE,
                          chrono.ChFrameD(chrono.ChVector3d(-10, -11, -9), chrono.Q_from_AngAxis(0, chrono.ChVector3D(0,0,1))))
sys.Add(link_revoluteE)

# Create a belt constraint between gear D and pulley E
link_belt = chrono.ChLinkBelt()
# Initialize with the bodies and connection points
# Define connection points relative to bodies
pointD = chrono.ChVector3d(0, 0, 0)  # Center of gear D
pointE = chrono.ChVector3d(0, 0, 0)  # Center of pulley E

link_belt.Initialize(mbody_gearD, mbody_pulleyE,
                     chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                       chrono.Q_identity()),
                     chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                       chrono.Q_identity()))
# Set parameters for belt tension, etc.
link_belt.SetTension(10)
sys.Add(link_belt)

# ------------------ Visualization of belt during scene rendering ------------------
# For visualization, create a visual shape or line between gear D and pulley E
# adding a custom visual shape or a debug line during rendering
# We'll create a visualization object outside the simulation loop

# Function to draw belt as a line (only in visualization)
def add_belt_visual(vis, body1, body2):
    pos1 = body1.GetPos()
    pos2 = body2.GetPos()
    vis.AddLine(pos1, pos2, chrono.ChColor(1, 0, 0))  # Red line

# ------------------ Setup visualization ------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys with belt')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Simulation parameters
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    # Optionally, draw belt as a line
    add_belt_visual(vis, mbody_gearD, mbody_pulleyE)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)