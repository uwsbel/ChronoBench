import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChMaterialSurfaceNSC()               # Corrected material creation for NSC contact method

# Create all rigid bodies with specific dimensions
radA = 1.5                                        # Modified radius for the first gear
radB = 3.5                                        # Modified radius for the second gear

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
                           chrono.ChFrameD(chrono.ChVectorD(0,0,0),     # Corrected to ChFrameD
                                           chrono.QUNIT))             # No initial rotation
sys.AddLink(link_revoluteTT)                                          # Adding the joint to the system

# Create the first gear
mbody_gearA = chrono.ChBodyEasyCylinder(                              # Creating a cylindrical gear with Y axis as the central axis
    radA, 0.5,                                                        # Setting radius and height (changed order)
    1000, True, False, mat)                                           # Setting mass, visualization, collision, and material
# Note: Chrono's ChBodyEasyCylinder signature is (radius, height, density, visualization, collision, contact material)
sys.Add(mbody_gearA)                                                  # Adding the gear to the system
mbody_gearA.SetPos(chrono.ChVectorD(0, 0, -1))                       # Positioning the gear at (0, 0, -1) using ChVectorD
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))                     # Rotating the gear by 90 degrees around X-axis, using Q_from_AngX
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)                # Applying the visual material to the gear

# Adding a thin cylinder only for visualization purpose
mshaft_shape = chrono.ChVisualShapeCylinder(radA * 0.3, 10)                                # Modified thin cylinder for visualization
mbody_gearA.AddVisualShape(mshaft_shape, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),     # Added ChFrameD, not ChFramed
                                                          chrono.Q_from_AngX(chrono.CH_C_PI_2)))  # Using corrected constants and methods

# Impose rotation speed on the first gear relative to the fixed truss
link_motor = chrono.ChLinkMotorRotationSpeed()                      # Creating a motor link to impose rotation
link_motor.Initialize(mbody_gearA, mbody_truss,                     # Initializing the motor with gear and truss
                      chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),     # Positioning the motor at origin
                                      chrono.QUNIT))                # No initial rotation
link_motor.SetSpeedFunction(chrono.ChFunction_Const(3))             # Corrected to ChFunction_Const for constant rotation speed
sys.AddLink(link_motor)                                             # Adding the motor link to the system

# Create the second gear
interaxis12 = radA + radB                                           # Calculating distance between the centers of two gears
mbody_gearB = chrono.ChBodyEasyCylinder(
    radB, 0.4,                                                      # Setting radius and height
    1000, True, False, mat)                                         # Setting mass, visualization, collision, and material
sys.Add(mbody_gearB)                                                # Adding the second gear to the system
mbody_gearB.SetPos(chrono.ChVectorD(interaxis12, 0, -2))           # Modified position of the second gear to (interaxis12, 0, -2)
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))                   # Rotating the second gear by 90 degrees around X-axis
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)              # Applying the visual material to the gear

# Fix second gear to the rotating bar with a revolute joint
link_revolute = chrono.ChLinkLockRevolute()                         # Creating a revolute joint
link_revolute.Initialize(mbody_gearB, mbody_train,                  # Initializing the joint with second gear and rotating bar
                         chrono.ChFrameD(chrono.ChVectorD(interaxis12, 0, 0), chrono.QUNIT))  # Using ChFrameD and ChVectorD
sys.AddLink(link_revolute)                                          # Adding the joint to system

# Create the gear constraint between the two gears, A and B
# For gear ratio the transmission ratio is set as radA/radB.
link_gearAB = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD())       # Using ChFrameD
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    # Setting frame for shaft1
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    # Setting frame for shaft2
link_gearAB.SetTransmissionRatio(radA / radB)                             # Setting transmission ratio as radA/radB
link_gearAB.SetEnforcePhase(True)                                         # Enforcing phase matching between gears
sys.AddLink(link_gearAB)                                                  # Adding the gear constraint to the system

# Create the gear constraint between second gear B and a large wheel C with inner teeth
# Note: here, the last wheel C is the truss itself
radC = 2 * radB + radA                                                    # Calculating radius for large wheel C
link_gearBC = chrono.ChLinkLockGear()                                     # Creating a gear constraint link
link_gearBC.Initialize(mbody_gearB, mbody_truss, chrono.ChFrameD())       # Using ChFrameD
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))    # Setting frame for second gear B shaft
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVectorD(0, 0, -4), chrono.QUNIT))        # Setting frame for large wheel C shaft
link_gearBC.SetTransmissionRatio(radB / radC)                             # Setting transmission ratio as radB/radC
link_gearBC.SetEpicyclic(True)                                            # Enabling epicyclic gear set (internal teeth)
sys.AddLink(link_gearBC)                                                  # Adding the gear constraint to the system

# --- Modifications start here ---

# 1. Add Bevel Gear (gear D)
radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(
    radD, 0.5,                                                      # Bevel gear thickness same as others, can adjust if wanted
    1000, True, False, mat)
sys.Add(mbody_gearD)
mbody_gearD.SetPos(chrono.ChVectorD(-10, 0, -9))
mbody_gearD.SetRot(chrono.Q_from_AngZ(m.pi / 2))                    # Rotate 90 degrees about Z-axis
mbody_gearD.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add a thin visual shaft for gear D similar to gear A for better visualization
mshaft_shape_D = chrono.ChVisualShapeCylinder(radD * 0.3, 10)
mbody_gearD.AddVisualShape(mshaft_shape_D, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),
                                                           chrono.Q_from_AngX(chrono.CH_C_PI_2)))

# Revolute joint between gear D and truss, allowing rotation along axis horizontal (X axis)
link_revolute_D_truss = chrono.ChLinkLockRevolute()
# Revolute joint frame positioned at gear D center
# The joint axis - since gearD is rotated 90 degrees about Z, horizontal axis is along X.
link_revolute_D_truss.Initialize(
    mbody_truss, mbody_gearD,
    chrono.ChFrameD(chrono.ChVectorD(-10, 0, -9), chrono.QUNIT))
sys.AddLink(link_revolute_D_truss)

# Gear constraint between gear A and gear D with 1:1 ratio
link_gearAD = chrono.ChLinkLockGear()
link_gearAD.Initialize(mbody_gearA, mbody_gearD, chrono.ChFrameD())
# Assuming shafts for A and D aligned along Y axis for gear A and
# rotated for gear D: we must set frames that reflect correct shaft directions.

# Gear A shaft frame: rotated -90deg about X, same as before
link_gearAD.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi / 2)))
# Gear D shaft frame: rotated -90deg about Z, since gear D axis is along X after rotation Z90
# We need the shaft axis aligned along Y after its rotation by Z90, so shaft axis is X, so rotate accordingly
# Check carefully: Gear D axis is along X after Z90 rotation of the cylinder along Y axis.
# So we must set the shaft frame for gear D along X (which is default axis for gear D after rotation).

# Since gearD: axis along X, no rotation needed for shaft frame on gear D (identity)
link_gearAD.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))

link_gearAD.SetTransmissionRatio(1.0)  # 1:1 ratio
link_gearAD.SetEnforcePhase(True)
sys.AddLink(link_gearAD)

# 2. Add Pulley (pulley E)
radE = 2
mbody_pulleyE = chrono.ChBodyEasyCylinder(
    radE, 0.4,                              # height adjusted slightly for pulley
    1000, True, False, mat)
sys.Add(mbody_pulleyE)
mbody_pulleyE.SetPos(chrono.ChVectorD(-10, -11, -9))
mbody_pulleyE.SetRot(chrono.Q_from_AngZ(m.pi / 2))                # Rotate 90 degrees around Z axis
mbody_pulleyE.GetVisualShape(0).SetMaterial(0, vis_mat)

# Add a shaft visual to pulley E for consistent look
mshaft_shape_E = chrono.ChVisualShapeCylinder(radE * 0.3, 10)
mbody_pulleyE.AddVisualShape(mshaft_shape_E, chrono.ChFrameD(chrono.ChVectorD(0, 3.5, 0),
                                                           chrono.Q_from_AngX(chrono.CH_C_PI_2)))

# Revolute joint between pulley E and truss along horizontal axis (X axis)
link_revolute_E_truss = chrono.ChLinkLockRevolute()
link_revolute_E_truss.Initialize(
    mbody_truss, mbody_pulleyE,
    chrono.ChFrameD(chrono.ChVectorD(-10, -11, -9), chrono.QUNIT))
sys.AddLink(link_revolute_E_truss)

# Synchro belt constraint between gear D and pulley E
# Chrono currently does not have a direct "synchro belt" constraint class in pychrono bindings
# Typically, a ChLinkDistance or a ChLinkGear with custom constraints can mimic belts.
# Alternatively, use a ChLinkGear for 1:1 ratio belt drive for synchronization.
link_belt_DE = chrono.ChLinkGear()
link_belt_DE.Initialize(mbody_gearD, mbody_pulleyE, chrono.ChFrameD())

# Set shaft frames to match axes of gear D and pulley E shafts:
# Both gear D and pulley E shafts are along X-axis due to Z rotation, so frame is identity
link_belt_DE.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))
link_belt_DE.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))

# Transmission ratio equal to ratio of radii (gear ratio for belt)
# For synchro belt, speed ratio = radius gear D / radius pulley E
link_belt_DE.SetTransmissionRatio(radD / radE)
link_belt_DE.SetEnforcePhase(True)
sys.AddLink(link_belt_DE)

# --- Visualization enhancement for belt ---

# We will add a simple visual line representing the belt linking gear D and pulley E
# Define a class to add belt visualization during rendering
class BeltVisual(chronoirr.ChIrrUpgrade):
    def __init__(self, vis_sys, body1, body2):
        super().__init__()
        self.vis_sys = vis_sys
        self.body1 = body1
        self.body2 = body2
        # Create line node in Irrlicht
        from pychrono import irrlicht as irr
        driver = self.vis_sys.GetDevice().getVideoDriver()
        smgr = self.vis_sys.GetDevice().getSceneManager()
        self.line_node = smgr.addAnimatedMeshSceneNode(irr.createLine([irr.vector3df(0,0,0), irr.vector3df(1,1,1)]))
        self.line_node.setMaterialFlag(irr.EMF_LIGHTING, False)
        self.line_node.setMaterialType(irr.EMT_TRANSPARENT_ADD_COLOR)
        self.line_node.setMaterialTexture(0, None)
        self.line_node.getMaterial(0).EmissiveColor = irr.SColor(255, 255, 255, 100)  # dim blue-ish
        self.line_node.setVisible(True)

    def Update(self):
        from pychrono import irrlicht as irr
        if not self.vis_sys.Run():
            return
        pos1 = self.body1.GetPos()
        pos2 = self.body2.GetPos()
        # Shift slightly outward along local axis for visual separation from shafts/radii
        # Get vector from gear D to pulley E
        dir_vec = pos2 - pos1
        # Update the line node vertices
        v0 = irr.vector3df(float(pos1.x), float(pos1.y), float(pos1.z))
        v1 = irr.vector3df(float(pos2.x), float(pos2.y), float(pos2.z))
        # Use a simple animated mesh line with 2 points
        # Since irrlicht does not provide a direct mesh line update,
        # we will remove old line node and add a new one each frame (not optimal but simple)
        # So remove old and create new each frame
        node = self.line_node
        smgr = self.vis_sys.GetDevice().getSceneManager()
        smgr.getRootSceneNode().removeChild(node)
        self.line_node.remove()
        # Add a new line scene node
        self.line_node = smgr.addAnimatedMeshSceneNode(irr.createLine([v0, v1]))
        self.line_node.setMaterialFlag(irr.EMF_LIGHTING, False)
        self.line_node.setMaterialType(irr.EMT_TRANSPARENT_ADD_COLOR)
        self.line_node.setMaterialTexture(0, None)
        self.line_node.getMaterial(0).EmissiveColor = irr.SColor(255, 255, 255, 150)
        self.line_node.setVisible(True)

# Alternatively, since the above is complex, we'll implement belt visual simply in main loop with irrlicht primitives:

# --- End modifications ---

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()                        # Initializing the Irrlicht visualization system
vis.AttachSystem(sys)                                           # Attaching the simulation system to visualization
vis.SetWindowSize(1024, 768)                                    # Setting window size for the visualization
vis.SetWindowTitle('Gears and pulleys')                         # Setting window title
vis.Initialize()                                                # Initializing the visualization window
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # Adding a logo to the visualization window
vis.AddSkyBox()                                                 # Adding a skybox for background
vis.AddCamera(chrono.ChVectorD(12, 15, -20))                   # Adding a camera at a specific position
vis.AddTypicalLights()                                          # Adding typical lights

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED) # Setting the type of integrator for time-stepping
sys.SetTimestep(1e-3)                                           # Set timestep explicitly

# Helper function to draw simple belt visualization between gear D and pulley E
def draw_belt(vis_sys, pos1, rad1, pos2, rad2):
    # Draw a line and two circles representing the belt path and pulleys
    # Irrlicht supports draw methods via scene manager
    # Get Irrlicht device stuff
    smgr = vis_sys.GetDevice().getSceneManager()
    driver = vis_sys.GetDevice().getVideoDriver()

    vec_start = chrono.ChVectorD(pos1.x, pos1.y, pos1.z)
    vec_end = chrono.ChVectorD(pos2.x, pos2.y, pos2.z)

    # Convert to irrlicht vector3df
    from pychrono import irrlicht as irr
    irr_start = irr.vector3df(float(pos1.x), float(pos1.y), float(pos1.z))
    irr_end = irr.vector3df(float(pos2.x), float(pos2.y), float(pos2.z))

    # Draw line - not persistent (drawn each frame)
    driver.setTransform(irr.ETS_WORLD, irr.matrix4())
    driver.draw3DLine(irr_start, irr_end, irr.SColor(255, 100, 100, 255))

    # Draw pulley circles at pos1 and pos2
    # Approximated using debug spheres with radius=rad (not cylinders)
    smgr.addSphereSceneNode(rad1*0.05, 12, None, -1, irr_start, irr.vector3df(1,1,1), irr.SColor(150,255,255,255))
    smgr.addSphereSceneNode(rad2*0.05, 12, None, -1, irr_end, irr.vector3df(1,1,1), irr.SColor(150,255,255,255))


# Simulation loop
while vis.Run():                                    # Running the visualization loop while the window is open
    vis.BeginScene()                                # Begin rendering a new scene
    vis.Render()                                    # Render the current simulation state

    # Draw belt visualization between gear D and pulley E
    draw_belt(vis, mbody_gearD.GetPos(), radD, mbody_pulleyE.GetPos(), radE)

    vis.EndScene()                                  # End rendering the scene
    sys.DoStepDynamics(1e-3)                        # Perform a simulation step with a time step of 1ms