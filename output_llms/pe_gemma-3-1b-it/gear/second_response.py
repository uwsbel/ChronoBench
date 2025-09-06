import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the system using Non-Smooth Contact (NSC) mechanics

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

# Create all rigid bodies with specific dimensions
radA = 2                                          # Defining radius for first gear
radB = 4                                          # Defining radius for second gear

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(20, 10, 2, 1000, True, False, mat)
sys.Add(mbody_truss)                              # Adding the truss to the physical system

mbody_truss.SetFixed(True)                        # Ensuring the truss is fixed
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 0))    # Setting the position of the truss to (0, 0, 0)
mbody_truss.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2)) # Rotating the truss by 90 degrees around X-axis

# Revolute joints
rev_joint = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
rev_joint.Initialize(mbody_truss, mbody_train,                  # Initializing the joint with truss and rotating bar
                         chrono.ChFramed(chrono.ChVector3d(0, 0, 0),     # Positioning the joint at origin
                                           chrono.QUNIT))             # No initial rotation

sys.AddLink(rev_joint)                                          # Adding the revolute joint to the system

# Spherical Joints
spherical_joint = chrono.ChLinkLockSpherical()                         # Creating a spherical joint
spherical_joint.Initialize(mbody_gearA, mbody_train,                  # Initializing the joint with gear and truss
                           chrono.ChFramed(chrono.ChVector3d(0, 0, 0),    # Positioning the joint at origin
                                           chrono.QUNIT))             # No initial rotation

sys.AddLink(spherical_joint)                                          # Adding the spherical joint to the system

# Universal Joints
univ_joint = chrono.ChLinkUniversal()                                     # Creating a universal joint
univ_joint.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed(chrono.ChVector3d(0, 0, 0),  # Positioning the joint at origin
                                           chrono.QUNIT))   # No initial rotation

sys.AddLink(univ_joint)                                          # Adding the universal joint to the system

# Motor Joints
motor = chrono.ChLinkMotorRotationSpeed()                         # Creating a motor link to impose rotation
motor.Initialize(mbody_gearB, mbody_train,                  # Initializing the motor with gear and truss
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),   # Positioning the motor at origin
                                      chrono.QUNIT))             # No initial rotation

sys.AddLink(motor)                                                  # Adding the motor link to the system

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             # Creating a cylindrical gear with Y axis as the central axis
                                        radA, 0.5,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearA)                                                # Adding the gear to the system
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                     # Positioning the gear at (0, 0, -1)

mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2)) # Rotating the gear by 90 degrees around X-axis

mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

# Add a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.4, 13)                                # Creating a thin cylinder with radius 0.4 and height 13
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),  # Positioning the cylinder at (0, 3.5, 0)
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))   # Positioning the cylinder at (0, 3.5, 0)

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                                     # Creating a motor link to impose rotation
link_motor.Initialize(mbody_gearB, mbody_train,                  # Initializing the motor with gear and truss
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -1),    # Positioning the motor at (interaxis12, 0, -1)
                                           chrono.QUNIT))             # No initial rotation

sys.AddLink(link_motor)                                          # Adding the motor link to the system

# Create the second gear
interaxis12 = radA + radB                                           # Calculating distance between the centers of two gears
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            # Creating second gear with cylinder shape
                                        radB, 0.4,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearB)                                                # Adding the second gear to the system
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           # Positioning the second gear based on calculated inter-axis distance

mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2)) # Rotating the second gear by 90 degrees around X-axis

mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

# Fix second gear to the rotating bar with a revolute joint
link_revolute = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revolute.Initialize(mbody_gearB, mbody_train,                  # Initializing the joint with second gear and rotating bar
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -1),    # Positioning the joint at (interaxis12, 0, -1)
                                           chrono.QUNIT))             # No initial rotation

sys.AddLink(link_revolute)                                          # Adding the revolute joint to the system

# Create the gear constraint between the two gears, A and B
# For gear ratio the transmission ratio is set as radA/radB.
link_gearAB = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, -4),  # Positioning the joint at (interaxis12, 0, -4)
                                           chrono.QUNIT))   # No initial rotation

sys.AddLink(link_gearAB)                                          # Adding the gear constraint to the system

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()                        # Initializing the Irrlicht visualization system
vis.AttachSystem(sys)                                           # Attaching the simulation system to visualization
vis.SetWindowSize(1024, 768)                                    # Setting window size for the visualization
vis.SetWindowTitle('Gears and pulleys')                         # Setting window title
vis.Initialize()                                                # Initializing the visualization window
vis.AddSkyBox()                                                 # Adding a skybox for background
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                   # Adding a camera at a specific position
vis.AddTypicalLights()                                          # Adding typical lights