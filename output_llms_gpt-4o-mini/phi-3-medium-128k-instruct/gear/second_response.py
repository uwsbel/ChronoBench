import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math as m

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()  # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()  # Creating a contact material for collision handling

# Create all rigid bodies with specific dimensions
radA = 1.5  # Defining radius for first gear
radB = 3.5  # Defining radius for second gear

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,  # Creating a box-shaped truss body with dimensions 15x8x2
                                   1000,          # Setting mass (not used for fixed body)
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_truss)  # Adding the truss to the physical system
mbody_truss.SetFixed(True)  # Making the truss fixed (immovable)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))  # Setting the position of the truss to (0, 0, 3)

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()  # Creating a visual material
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  # Setting a texture for the visual material

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  # Creating a box-shaped rotating bar with dimensions 8x1.5x1.0
                                   1000,          # Setting mass
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_train)  # Adding the rotating bar to the system
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))  # Positioning the rotating bar at (3, 0, 0)

# Create a revolute joint between truss and rotating bar, allowing rotation along the Z-axis
link_revoluteTT = chrono.ChLinkLockRevolute()  # Creating a revolute joint
link_revoluteTT.Initialize(mbody_truss, mbody_train,  # Initializing the joint with truss and rotating bar
                           chrono.ChFramed(chrono.ChVector3d(0,0,0),  # Positioning the joint at origin
                                           chrono.QUNIT))             # No initial rotation
sys.AddLink(link_revoluteTT)  # Adding the joint to the system