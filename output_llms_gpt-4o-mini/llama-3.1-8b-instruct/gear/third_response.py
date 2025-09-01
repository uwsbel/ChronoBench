"""
Importing necessary libraries
"""
import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

"""
Create a Chrono::Engine physical system
"""
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

"""
Create contact material shared among all bodies
"""
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

"""
Create all rigid bodies with specific dimensions
"""
radA = 1.5                                        # Modified radius for the first gear
radB = 3.5                                        # Modified radius for the second gear
radC = 2 * radB + radA                            # Calculating radius for large wheel C
radD = 5                                          # Radius for the bevel gear
radE = 2                                          # Radius for the pulley

"""
Create the truss
"""
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      # Modified box-shaped truss body with dimensions 15x8x2
                                   1000,          # Setting mass (not used for fixed body)
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_truss)                              # Adding the truss to the physical system
mbody_truss.SetFixed(True)                        # Making the truss fixed (immovable)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))    # Setting the position of the truss to (0, 0, 3)

"""
Shared visualization material for enhanced aesthetics
"""
vis_mat = chrono.ChVisualMaterial()                       # Creating a visual material
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  # Setting a texture for the visual material

"""
Create the rotating bar support for the two epicycloidal wheels
"""
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  # Creating a box-shaped rotating bar with dimensions 8x1.5x1.0
                                   1000,          # Setting mass
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_train)                              # Adding the rotating bar to the system
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))    # Positioning the rotating bar at (3, 0, 0)

"""
Create a revolute joint between truss and rotating bar, allowing rotation along the Z-axis
"""
link_revoluteTT = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteTT.Initialize(mbody_truss, mbody_train,                  # Initializing the joint with truss and rotating bar
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),  # Positioning the joint at origin
                                           chrono.QUNIT))             # No initial rotation
sys.AddLink(link_revoluteTT)                                          # Adding the joint to the system

"""
Create the first gear
"""
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             # Creating a cylindrical gear with Y axis as the central axis
                                        radA, 0.5,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearA)                                                # Adding the gear to the system
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))                     # Positioning the gear at (0, 0, -1)
mbody_gearA.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # Rotating the gear by 90 degrees around X-axis
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

"""
Adding a thin cylinder only for visualization purpose
"""
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                # Modified thin cylinder for visualization
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     # Adding the visual shape to the gear body
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  # Positioning and rotating the visual cylinder

"""
Impose rotation speed on the first gear relative to the fixed truss
"""
link_motor = chrono.ChLinkMotorRotationSpeed()                      # Creating a motor link to impose rotation
link_motor.Initialize(mbody_gearA, mbody_truss,                     # Initializing the motor with gear and truss
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0),   # Positioning the motor at origin
                                      chrono.QUNIT))                # No initial rotation
link_motor.SetSpeedFunction(chrono.ChFunctionConst(3))              # Modified constant rotation speed to 3 rad/s
sys.AddLink(link_motor)                                             # Adding the motor link to the system

"""
Create the second gear
"""
interaxis12 = radA + radB                                           # Calculating distance between the centers of two gears
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,            # Creating second gear with cylinder shape
                                        radB, 0.4,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearB)                                                # Adding the second gear to the system
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))           # Modified position of the second gear to (interaxis12, 0, -2)
mbody_gearB.SetRot(chrono.QuatFromAngleX(m.pi / 2))                 # Rotating the second gear by 90 degrees around X-axis
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

"""
Fix second gear to the rotating bar with a revolute joint
"""
link_revolute = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revolute.Initialize(mbody_gearB, mbody_train,                  # Initializing the joint with second gear and rotating bar
                         chrono.ChFramed(chrono.ChVector3d(interaxis12, 0, 0), chrono.QUNIT))  # Positioning the joint at the appropriate inter-axis distance
sys.AddLink(link_revolute)                                          # Adding the joint to the system

"""
Create the gear constraint between the two gears, A and B
"""
link_gearAB = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFramed())       # Initializing the gear link between gear A & B
link_gearAB.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft1
link_gearAB.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft2
link_gearAB.SetTransmissionRatio(radA / radB)                             # Setting transmission ratio as radA/radB
link_gearAB.SetEnforcePhase(True)                                         # Enforcing phase matching between gears
sys.AddLink(link_gearAB)                                                  # Adding the gear constraint to the system

"""
Create the gear constraint between second gear B and a large wheel C with inner teeth
"""
link_gearBC = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFramed())       # Initializing the gear link between gear B & truss
link_gearBC.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for second gear B shaft
link_gearBC.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0, 0, -4), chrono.QUNIT))        # Setting frame for large wheel C shaft
link_gearBC.SetTransmissionRatio(radB / radC)                             # Setting transmission ratio as radB/radC
link_gearBC.SetEpicyclic(True)                                            # Enabling epicyclic gear set (internal teeth)
sys.AddLink(link_gearBC)                                                  # Adding the gear constraint to the system

"""
Create the bevel gear
"""
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             # Creating a cylindrical gear with Y axis as the central axis
                                        radD, 0.5,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_gearD)                                                # Adding the gear to the system
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))                     # Positioning the gear at (-10, 0, -9)
mbody_gearD.SetRot(chrono.QuatFromAngleZ(m.pi / 2))                  # Rotating the gear by 90 degrees around Z-axis
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the gear

"""
Adding a visual shape for the bevel gear
"""
mshaft_shape = chrono.ChVisualShapeCylinder(radD * 0.3, 10)                                # Modified thin cylinder for visualization
mbody_gearD.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     # Adding the visual shape to the gear body
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  # Positioning and rotating the visual cylinder

"""
Create a revolute joint between truss and bevel gear, allowing rotation along the X-axis
"""
link_revoluteDD = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteDD.Initialize(mbody_gearD, mbody_truss,                  # Initializing the joint with bevel gear and truss
                           chrono.ChFramed(chrono.ChVector3d(-10, 0, -9),  # Positioning the joint at the gear position
                                           chrono.QUNIT))             # No initial rotation
sys.AddLink(link_revoluteDD)                                          # Adding the joint to the system

"""
Create a gear constraint between gear A and gear D
"""
link_gearAD = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFramed())       # Initializing the gear link between gear A & D
link_gearAD.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft1
link_gearAD.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft2
link_gearAD.SetTransmissionRatio(radA / radD)                             # Setting transmission ratio as radA/radD
link_gearAD.SetEnforcePhase(True)                                         # Enforcing phase matching between gears
sys.AddLink(link_gearAD)                                                  # Adding the gear constraint to the system

"""
Create the pulley
"""
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,             # Creating a cylindrical pulley with Y axis as the central axis
                                        radE, 0.5,                  # Setting radius and height
                                        1000, True, False, mat)     # Setting mass, visualization, collision, and material
sys.Add(mbody_pulleyE)                                                # Adding the pulley to the system
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))                     # Positioning the pulley at (-10, -11, -9)
mbody_pulleyE.SetRot(chrono.QuatFromAngleZ(m.pi / 2))                  # Rotating the pulley by 90 degrees around Z-axis
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)               # Applying the visual material to the pulley

"""
Adding a visual shape for the pulley
"""
mshaft_shape = chrono.ChVisualShapeCylinder(radE * 0.3, 10)                                # Modified thin cylinder for visualization
mbody_pulleyE.AddVisualShape(mshaft_shape, chrono.ChFramed(chrono.ChVector3d(0, 3.5, 0),     # Adding the visual shape to the pulley body
                                                          chrono.QuatFromAngleX(chrono.CH_PI_2)))  # Positioning and rotating the visual cylinder

"""
Create a revolute joint between truss and pulley, allowing rotation along the X-axis
"""
link_revoluteEE = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revoluteEE.Initialize(mbody_pulleyE, mbody_truss,                  # Initializing the joint with pulley and truss
                           chrono.ChFramed(chrono.ChVector3d(-10, -11, -9),  # Positioning the joint at the pulley position
                                           chrono.QUNIT))             # No initial rotation
sys.AddLink(link_revoluteEE)                                          # Adding the joint to the system

"""
Create a synchro belt constraint between gear D and pulley E
"""
link_beltDE = chrono.ChLinkSynchroBelt()                                     # Creating a synchro belt link
link_beltDE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFramed())       # Initializing the belt link between gear D & pulley E
link_beltDE.SetFrameShaft1(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft1
link_beltDE.SetFrameShaft2(chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(-m.pi / 2)))    # Setting frame for shaft2
link_beltDE.SetTransmissionRatio(radD / radE)                             # Setting transmission ratio as radD/radE
sys.AddLink(link_beltDE)                                                  # Adding the belt constraint to the system

"""
Create the Irrlicht visualization
"""
vis = chronoirr.ChVisualSystemIrrlicht()                        # Initializing the Irrlicht visualization system
vis.AttachSystem(sys)                                           # Attaching the simulation system to visualization
vis.SetWindowSize(1024, 768)                                    # Setting window size for the visualization
vis.SetWindowTitle('Gears and pulleys')                         # Setting window title
vis.Initialize()                                                # Initializing the visualization window
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # Adding a logo to the visualization window
vis.AddSkyBox()                                                 # Adding a skybox for background
vis.AddCamera(chrono.ChVector3d(12, 15, -20))                   # Adding a camera at a specific position
vis.AddTypicalLights()                                          # Adding typical lights

"""
Set integrator type
"""
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) # Setting the type of integrator for time-stepping

"""
Simulation loop
"""
while vis.Run():                                    # Running the visualization loop while the window is open
    vis.BeginScene()                                # Begin rendering a new scene
    vis.Render()                                    # Render the current simulation state
    vis.EndScene()                                  # End rendering the scene
    sys.DoStepDynamics(1e-3)                        # Perform a simulation step with a time step of 1ms