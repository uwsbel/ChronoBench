import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the system using Non-Smooth Contact (NSC) mechanics

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

# Create all rigid bodies with specific dimensions
radA = 1.5                                        # Modified radius for the first gear
radB = 3.5                                        # Modified radius for the second gear

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      # Modified box-shaped truss body with dimensions 15x8x2
                                   1000,          # Setting mass (not used for fixed body)
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material

sys.Add(mbody_truss)                              # Adding the truss to the system

mbody_truss.SetFixed(True)                        # Making the truss fixed (immovable)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 0))    # Setting the position of the truss to (0, 0, 0)
mbody_truss.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))    # Rotating the truss by 90 degrees around the X-axis
mbody_truss.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the truss

# Create the revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  # Initializing the joint with truss and rotating bar
                         chrono.ChFramed(chrono.ChVector3d(0, 0, 0),     # Positioning the joint at origin
                                           chrono.QUNIT))             # No initial rotation
sys.AddLink(link_revoluteTT)                                          # Adding the joint to the system

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            # Creating a cylindrical gear with Y axis as the central axis
                                        radA, 0.5,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearA)                                                # Adding the gear to the system
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))    # Modified position of the first gear to (0, 0, -1)
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # Rotating the first gear by 90 degrees around the X-axis
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

# Add a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                # Modified thin cylinder for visualization
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     # Adding the visual shape to the gear
                                                          chrono.QuatFromAngleX(-m.pi / 2)))    # Positioning the visual cylinder

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                                     # Creating a motor link to impose rotation
link_motor.Initialize(mbody_gearA, mbody_train,                  # Initializing the motor with gear and truss
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),   # Positioning the motor at origin
                                      chrono.QUNIT))                # No initial rotation
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))              # Modified constant rotation speed to 3 rad/s
sys.AddLink(link_motor)                                                  # Adding the motor link to the system

# Create the second gear
interaxis12 = radA + radB                                           # Calculating distance between the centers of two gears
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            # Creating second gear with cylinder shape
                                        radB, 0.4,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearB)                                                # Adding the second gear to the system
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           # Modified position of the second gear to (interaxis12, 0, -2)
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # Rotating the second gear by 90 degrees around the X-axis
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

# Add a pulley
mbody_pulley = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            # Creating a cylinder for the pulley
                                        radA, 0.3,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_pulley)                                                # Adding the pulley to the system

# Create the first gear constraint between the two gears, A and B
# For gear ratio the transmission ratio is set as radA/radB.
link_gearAB = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearAB.Initialize(mbody_gearA, mbody_train, chrono.ChFramed())       # Initializing the gear link between gear A & B
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft1
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for large wheel C
link_gearAB.SetTransmissionRatio(radA / radB)                             # Setting transmission ratio as radA/radB
link_gearAB.SetEnforcePhase(True)                                            # Enforcing phase matching between gears
sys.AddLink(link_gearAB)                                                  # Adding the gear constraint to the system

# Create the gear constraint between second gear B and a large wheel C with inner teeth
# Note: here, the last wheel C is the truss itself
radC = 2 * radB + radA                                                    # Calculating radius for large wheel C
link_gearBC = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())       # Initializing the gear link between gear B & truss
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for second gear B shaft
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for large wheel C shaft
link_gearBC.SetTransmissionRatio(radB / radC)                             # Setting transmission ratio as radB/radC
link_gearBC.SetEpicyclic(True)                                            # Enabling epicyclic gear set (internal teeth)
sys.AddLink(link_gearBC)                                                  # Adding the gear constraint to the system

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()                        # Initializing the Irrlicht visualization system
vis.AttachSystem(sys)                                           # Attaching the simulation system to visualization
vis.SetWindowSize(1024, 768)                                    # Setting window size for the visualization
vis.SetWindowTitle('Gears and pulleys')                         # Setting window title
vis.Initialize()                                                # Initializing the visualization window
vis.AddSkyBox()                                                 # Adding a skybox for background
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                   # Adding a camera at a specific position
vis.AddTypicalLights()                                          # Adding typical lights

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) # Setting the type of integrator for time-stepping