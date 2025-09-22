import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling
# Properties for contact material (optional, can be set if collision is enabled)
# mat.SetFriction(0.5)
# mat.SetRestitution(0.1)

# Create all rigid bodies with specific dimensions
radA = 1.5                                        # Radius for the first gear
radB = 3.5                                        # Radius for the second gear

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      # Box-shaped truss body with dimensions 15x8x2 (halflengths)
                                   1000,          # Density (kg/m^3)
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
                                   1000,          # Setting density
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_train)                              # Adding the rotating bar to the system
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))    # Positioning the rotating bar at (3, 0, 0)
mbody_train.GetVisualShape(0).SetMaterial(0, vis_mat) # Apply visual material

# Create a revolute joint between truss and rotating bar, allowing rotation along the Z-axis
link_revoluteTT = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  # Initializing the joint with truss and rotating bar
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),  # Positioning the joint at origin (relative to truss)
                                           chrono.QUNIT))             # No initial rotation (joint Z along global Z)
sys.AddLink(link_revoluteTT)                                          # Adding the joint to the system

# Create the first gear
gear_height = 0.5
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.Y_AXIS,             # Creating a cylindrical gear with Y axis as the central axis
                                        radA, gear_height,         # Setting radius and height
                                        1000, True, False, mat)    # Setting density, visualization, collision, and material
sys.Add(mbody_gearA)                                               # Adding the gear to the system
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                    # Positioning the gear at (0, 0, -1)
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                # Rotating the gear by 90 degrees around X-axis (so local Y is global Z)
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)              # Applying the visual material to the gear

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                # Thin cylinder for visualization
mshaft_frame = chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)) # Position relative to gear A
# mbody_gearA.AddVisualShape(mshaft_shape, mshaft_frame) # This was causing visual issues, potentially related to orientation.
                                                        # Better to add shafts explicitly if needed. The main gear body is visualized.

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                      # Creating a motor link to impose rotation
# Motor frame on gear A's axis of rotation
motor_frame_abs = chrono.ChFramed(mbody_gearA.GetPos(), chrono.QUNIT) # Motor Z axis along global Z, at gear A's center
link_motor.Initialize(mbody_gearA, mbody_truss, motor_frame_abs)
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))              # Constant rotation speed to 3 rad/s
sys.AddLink(link_motor)                                             # Adding the motor link to the system

# Create the second gear
interaxis12 = radA + radB                                           # Calculating distance between the centers of two gears
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.Y_AXIS,              # Creating second gear with cylinder shape
                                        radB, 0.4,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting density, visualization, collision, and material
sys.Add(mbody_gearB)                                                # Adding the second gear to the system
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           # Position of the second gear
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # Rotating the second gear by 90 degrees around X-axis
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

# Fix second gear to the rotating bar with a revolute joint
# Joint frame: at gearB's position, axis along gearB's local Y (which is global Z)
joint_frame_B_train = chrono.ChFramed(mbody_gearB.GetPos(), chrono.QUNIT)
link_revolute = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revolute.Initialize(mbody_gearB, mbody_train, joint_frame_B_train)
sys.AddLink(link_revolute)                                          # Adding the joint to the system

# Create the gear constraint between the two gears, A and B
# Shafts are parallel (Global Z).
# Frame for ChLinkLockGear: Z axis is the common shaft direction (Global Z). Origin can be anywhere on this line.
gear_AB_common_frame = chrono.ChFramed(chrono.VNULL, chrono.QUNIT)
link_gearAB = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearAB.Initialize(mbody_gearA, mbody_gearB, gear_AB_common_frame)    # Initializing the gear link
# Shaft frame on body: Z-axis of this frame is shaft axis in body coords.
# Gears use Y_AXIS, so rotate ChFrame by X_AXIS for Z_frame to become Y_body.
q_shaft_rot = chrono.QuatFromAngleX(m.pi / 2)
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, q_shaft_rot))    # Shaft on gearA (local Y)
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, q_shaft_rot))    # Shaft on gearB (local Y)
link_gearAB.SetTransmissionRatio(radA / radB)                             # Setting transmission ratio as radA/radB
link_gearAB.SetEnforcePhase(True)                                         # Enforcing phase matching between gears
sys.AddLink(link_gearAB)                                                  # Adding the gear constraint to the system

# Create the gear constraint between second gear B and a large wheel C with inner teeth
# Note: here, the last wheel C is the truss itself
radC = 2 * radB + radA                                                    # Calculating radius for large wheel C
# Common frame for gearBC: Global Z axis shaft direction
gear_BC_common_frame = chrono.ChFramed(chrono.VNULL, chrono.QUNIT) # Z along global Z
link_gearBC = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearBC.Initialize(mbody_gearB, mbody_truss, gear_BC_common_frame)    # Initializing the gear link
link_
print("error happened with only start ```python")