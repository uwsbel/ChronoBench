import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

# Create all rigid bodies with specific dimensions
radA = 1.5                                        # Radius for the first gear
radB = 3.5                                        # Radius for the second gear

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      # Box-shaped truss body with dimensions 15x8x2
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
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                # Thin cylinder for visualization
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     # Adding the visual shape to the gear body
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  # Positioning and rotating the visual cylinder

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                      # Creating a motor link to impose rotation
link_motor.Initialize(mbody_gearA, mbody_truss,                     # Initializing the motor with gear and truss
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),   # Positioning the motor at origin
                                      chrono.QUNIT))                # No initial rotation
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))              # Constant rotation speed of 3 rad/s
sys.AddLink(link_motor)                                             # Adding the motor link to the system

# Create the second gear
interaxis12 = radA + radB                                           # Calculating distance between the centers of two gears
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            # Creating second gear with cylinder shape
                                        radB, 0.4,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearB)                                                # Adding the second gear to the system
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           # Position of the second gear
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # Rotating the second gear by 90 degrees around X-axis
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

# Fix second gear to the rotating bar with a revolute joint
link_revolute = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revolute.Initialize(mbody_gearB, mbody_train,                  # Initializing the joint with second gear and rotating bar
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))  # Positioning the joint
sys.AddLink(link_revolute)                                          # Adding the joint to the system

# Create the gear constraint between the two gears, A and B
link_gearAB = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())       # Initializing the gear link between gear A & B
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft1
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft2
link_gearAB.SetTransmissionRatio(radA / radB)                             # Setting transmission ratio as radA/radB
link_gearAB.SetEnforcePhase(True)                                         # Enforcing phase matching between gears
sys.AddLink(link_gearAB)                                                  # Adding the gear constraint to the system

# Create the gear constraint between second gear B and a large wheel C with inner teeth
radC = 2 * radB + radA                                                    # Calculating radius for large wheel C
link_gearBC = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())       # Initializing the gear link between gear B & truss
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for second gear B shaft
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))         # Setting frame for large wheel C shaft
link_gearBC.SetTransmissionRatio(radB / radC)                             # Setting transmission ratio as radB/radC
link_gearBC.SetEpicyclic(True)                                            # Enabling epicyclic gear set (internal teeth)
sys.AddLink(link_gearBC)                                                  # Adding the gear constraint to the system

# ============================================================
# Add Bevel Gear D (radius = 5)
# ============================================================
radD = 5.0                                                           # Radius for bevel gear D

mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             # Creating bevel gear D as a cylinder
                                        radD, 0.5,                   # Radius and height
                                        1000, True, False, mat)      # Mass, visualization, collision, material
sys.Add(mbody_gearD)                                                 # Adding gear D to the system
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))                    # Position at (-10, 0, -9)
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))                  # Rotate 90 degrees around Z-axis
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)                # Apply visual material

# Add shaft visualization for gear D
mshaftD_shape = chrono.ChVisualShapeCylinder(radD * 0.3, 10)         # Thin shaft cylinder for gear D
mbody_gearD.AddVisualShape(mshaftD_shape,                            # Add visual shape
                           chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                           chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Revolute joint for gear D along horizontal axis (X-axis)
link_revoluteD = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteD.Initialize(mbody_gearD, mbody_truss,                  # Link gear D to truss
                          chrono.ChFramed(chrono.ChVector3d(-10, 0, -9),
                                          chrono.QuatFromAngleY(m.pi / 2)))  # Frame with Z along global X
sys.AddLink(link_revoluteD)                                          # Add joint to system

# Bevel gear constraint between A and D (1:1 gear ratio)
link_gearAD = chrono.ChLinkLockGear()                                # Creating bevel gear constraint
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())  # Initialize between gear A and gear D
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL,             # Shaft1 frame: along local Y of gear A (global Z)
                                            chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.VNULL,             # Shaft2 frame: along local Y of gear D (global -X)
                                            chrono.QuatFromAngleX(-m.pi / 2)))
link_gearAD.SetTransmissionRatio(-1)                                 # 1:1 bevel gear ratio (negative for bevel direction)
sys.AddLink(link_gearAD)                                             # Add bevel gear constraint to system

# ============================================================
# Add Pulley E (radius = 2)
# ============================================================
radE = 2.0                                                           # Radius for pulley E

mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,           # Creating pulley E as a cylinder
                                          radE, 0.5,                 # Radius and height
                                          1000, True, False, mat)    # Mass, visualization, collision, material
sys.Add(mbody_pulleyE)                                               # Adding pulley E to the system
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))                # Position at (-10, -11, -9)
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))                # Rotate 90 degrees around Z-axis
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)              # Apply visual material

# Add shaft visualization for pulley E
mshaftE_shape = chrono.ChVisualShapeCylinder(radE * 0.3, 10)         # Thin shaft cylinder for pulley E
mbody_pulleyE.AddVisualShape(mshaftE_shape,                          # Add visual shape
                             chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),
                                             chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Revolute joint for pulley E along horizontal axis (X-axis)
link_revoluteE = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,                # Link pulley E to truss
                          chrono.ChFramed(chrono.ChVector3d(-10, -11, -9),
                                          chrono.QuatFromAngleY(m.pi / 2)))  # Frame with Z along global X
sys.AddLink(link_revoluteE)                                          # Add joint to system

# Synchronous belt constraint between gear D and pulley E
link_beltDE = chrono.ChLinkLockGear()                                # Using gear constraint to model synchro belt
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed()) # Initialize between D and E
link_beltDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL,             # Shaft1 frame along rotation axis of D
                                            chrono.QuatFromAngleX(-m.pi / 2)))
link_beltDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL,             # Shaft2 frame along rotation axis of E
                                            chrono.QuatFromAngleX(-m.pi / 2)))
link_beltDE.SetTransmissionRatio(radD / radE)                        # Belt ratio: ω_E = ω_D * (radD/radE), same direction
sys.AddLink(link_beltDE)                                             # Add belt constraint to system

# ============================================================
# Create the Irrlicht visualization
# ============================================================
vis = chronoirr.ChVisualSystemIrrlicht()                        # Initializing the Irrlicht visualization system
vis.AttachSystem(sys)                                           # Attaching the simulation system to visualization
vis.SetWindowSize(1024, 768)                                    # Setting window size for the visualization
vis.SetWindowTitle('Gears and pulleys')                         # Setting window title
vis.Initialize()                                                # Initializing the visualization window
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # Adding a logo to the visualization window
vis.AddSkyBox()                                                 # Adding a skybox for background
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                   # Adding a camera at a specific position
vis.AddTypicalLights()                                          # Adding typical lights

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) # Setting the type of integrator for time-stepping

# Simulation loop
while vis.Run():                                    # Running the visualization loop while the window is open
    vis.BeginScene()                                # Begin rendering a new scene
    vis.Render()                                    # Render the current simulation state

    # Draw simplified belt visualization between gear D and pulley E
    posD = mbody_gearD.GetPos()                     # Get current position of gear D
    posE = mbody_pulleyE.GetPos()                   # Get current position of pulley E

    # Draw two belt strands (left and right tangent lines)
    chronoirr.drawSegment(vis,
                          posD + chrono.ChVector3d(0, 0, radD),
                          posE + chrono.ChVector3d(0, 0, radE),
                          chrono.ChColor(0.8, 0.6, 0.0), True)   # Belt strand 1 (front)
    chronoirr.drawSegment(vis,
                          posD + chrono.ChVector3d(0, 0, -radD),
                          posE + chrono.ChVector3d(0, 0, -radE),
                          chrono.ChColor(0.8, 0.6, 0.0), True)   # Belt strand 2 (back)

    vis.EndScene()                                  # End rendering the scene
    sys.DoStepDynamics(1e-3)                        # Perform a simulation step with a time step of 1ms