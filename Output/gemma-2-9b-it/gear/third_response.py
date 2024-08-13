import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

# Create all rigid bodies with specific dimensions
radA = 1.5                                        # Modified radius for the first gear
radB = 3.5                                        # Modified radius for the second gear
radD = 5                         # Radius for the bevel gear
radE = 2                         # Radius for the pulley

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      # Modified box-shaped truss body with dimensions 15x8x2
                                   1000,          # Setting mass (not used for fixed body)
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_truss)                              # Adding the truss to the physical system
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
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),  # Positioning the joint at origin
                                           chrono.QUNIT))             # No initial rotation
sys.AddLink(link_revoluteTT)                                          # Adding the joint to the system

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             # Creating a cylindrical gear with Y axis as the central axis
                                        radA, 0.5,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearA)                                                # Adding the gear to the system
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                     # Positioning the gear at (0, 0, -1)
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # Rotating the gear by 90 degrees around X-axis
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                # Modified thin cylinder for visualization
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     # Adding the visual shape to the gear body
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  # Positioning and rotating the visual cylinder

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                      # Creating a motor link to impose rotation
link_motor.Initialize(mbody_gearA, mbody_truss,                     # Initializing the motor with gear and truss
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),   # Positioning the motor at origin
                                      chrono.QUNIT))                # No initial rotation
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))              # Modified constant rotation speed to 3 rad/s
sys.AddLink(link_motor)                                             # Adding the motor link to the system

# Create the second gear
interaxis12 = radA + radB                                           # Calculating distance between the
print("error happened with only start ```python")