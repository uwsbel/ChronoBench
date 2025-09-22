import pychrono as chrono
import pychrono.irrlicht as irr
import math
import os

# ---------------------------------------------------------------------
# Set the path to the Chrono data files (textures, CAD models, etc.)
# ---------------------------------------------------------------------
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', 'chrono_data/'))
# Or, if you know the exact path:
# chrono.SetChronoDataPath('C:/path/to/chrono/data/') # Windows example
# chrono.SetChronoDataPath('/path/to/chrono/data/')   # Linux/macOS example


# ---------------------------------------------------------------------
# 1. Initialize PyChrono environment and core components
# ---------------------------------------------------------------------
print("Copyright (c) 2017 projectchrono.org")

# Create a Chrono physical system
system = chrono.ChSystemNSC() # Use Non-Smooth Contact (NSC) solver

# Set gravity (optional, but good practice)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Solver settings (optional, defaults are often fine)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # A fast solver
system.SetSolverMaxIterations(150)
system.SetTimestep(0.005) # Simulation time step

# ---------------------------------------------------------------------
# 2. Define Mechanism Parameters
# ---------------------------------------------------------------------
# Dimensions
crank_radius = 0.1  # [m]
conrod_length = 0.25 # [m]
piston_mass = 0.5    # [kg]
crank_mass = 0.2     # [kg]
conrod_mass = 0.3    # [kg]
element_thickness = 0.02 # [m] for visualization

# Motor speed
motor_speed_rpm = 60.0 # revolutions per minute
motor_speed_rad_s = motor_speed_rpm * (2 * math.pi) / 60.0

# Initial crank angle (radians)
initial_crank_angle = 0.0 # Start horizontally

# ---------------------------------------------------------------------
# 3. Add physical systems and objects
# ---------------------------------------------------------------------

# --- Create the Floor (Truss) ---
# This is a fixed body to which other parts are jointed.
floor = chrono.ChBodyEasyBox(1.0, 0.1, 0.5, 1000, True, True, None) # width, height, depth, density, visualize, collide
floor.SetPos(chrono.ChVector3d(0, -0.05, 0)) # Position it slightly below origin
floor.SetBodyFixed(True)
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
system.Add(floor)

# --- Create the Crankshaft ---
crank_pos_x = 0
crank_pos_y = 0 # Relative to its rotation axis
crank_pos_z = 0.0 # Slight offset to avoid Z-fighting with other coplanar elements

crankshaft = chrono.ChBodyEasyBox(crank_radius * 2, element_thickness, element_thickness, 1000, True, True, None)
crankshaft.SetMass(crank_mass)
# Initial position of the crankshaft's CoG (center of gravity)
# If the crank rotates around one end, its CoG will be at crank_radius from that end.
# For simplicity, we'll model it as a bar rotating around its center and place the conrod pin offset.
# Let's define the crank's rotation point:
crank_rotation_point = chrono.ChVector3d(0, 0, crank_pos_z)
crankshaft.SetPos(crank_rotation_point + chrono.ChVector3d(crank_radius * math.cos(initial_crank_angle),
                                                          crank_radius * math.sin(initial_crank_angle),
                                                          0))
crankshaft.SetRot(chrono.Q_from_AngZ(initial_crank_angle)) # Initial orientation
crankshaft.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
crankshaft.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.3, 0.8)) # Bluish
system.Add(crankshaft)

# --- Create the Connecting Rod ---
conrod = chrono.ChBodyEasyBox(conrod_length, element_thickness * 0.8, element_thickness * 0.8, 1000, True, True, None)
conrod.SetMass(conrod_mass)
# Initial position calculations
# Crank pin position (where conrod connects to crank)
crank_pin_x = crank_rotation_point.x + crank_radius * math.cos(initial_crank_angle)
crank_pin_y = crank_rotation_point.y + crank_radius * math.sin(initial_crank_angle)
crank_pin_z = crank_pos_z

# Piston pin position (where conrod connects to piston) - assuming piston moves along X-axis
# Using law of cosines or simple geometry for initial configuration
# x_piston = x_crank_pin + sqrt(L_conrod^2 - y_crank_pin^2)
# For simplicity, we'll let Chrono's solver assemble it from joint constraints.
# We'll initially place the CoG of the conrod appropriately.
conrod_angle = math.asin(-crank_radius * math.sin(initial_crank_angle) / conrod_length)
conrod_cog_x = crank_pin_x + (conrod_length / 2) * math.cos(conrod_angle)
conrod_cog_y = crank_pin_y + (conrod_length / 2) * math.sin(conrod_angle)

conrod.SetPos(chrono.ChVector3d(conrod_cog_x, conrod_cog_y, crank_pos_z))
conrod.SetRot(chrono.Q_from_AngZ(conrod_angle))
conrod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/ลายไม้.jpg')) # Wood texture
conrod.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.4, 0.2)) # Brownish
system.Add(conrod)

# --- Create the Piston ---
piston = chrono.ChBodyEasyBox(element_thickness * 2.5, element_thickness * 2.5, element_thickness * 2.5, 1000, True, True, None)
piston.SetMass(piston_mass)
# Initial piston position
# Assuming horizontal slider, piston y = crank_rotation_point.y
piston_x = crank_pin_x + conrod_length * math.cos(conrod_angle)
piston_y = crank_rotation_point.y # Piston slides horizontally at this y-level
piston_z = crank_pos_z

piston.SetPos(chrono.ChVector3d(piston_x, piston_y, piston_z))
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/rock.jpg'))
piston.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.6)) # Greyish
system.Add(piston)


# ---------------------------------------------------------------------
# 4. Add Joints
# ---------------------------------------------------------------------

# --- Revolute Joint: Floor <-> Crankshaft ---
# Joint location in absolute coordinates
joint_crank_floor_pos = crank_rotation_point
# Joint axis (Z-axis)
joint_axis = chrono.ChVector3d(0, 0, 1)

rev_crank_floor = chrono.ChLinkRevolute()
rev_crank_floor.Initialize(crankshaft,         # Body 1
                           floor,              # Body 2
                           True,               # Use absolute coordinates for positioning
                           chrono.ChFrameD(joint_crank_floor_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X))) # Joint frame, aligned with Z for rotation
system.AddLink(rev_crank_floor)

# --- Revolute Joint: Crankshaft <-> Connecting Rod ---
# Joint location (crank pin) in absolute coordinates
joint_crank_conrod_pos = chrono.ChVector3d(crank_pin_x, crank_pin_y, crank_pin_z)

rev_crank_conrod = chrono.ChLinkRevolute()
rev_crank_conrod.Initialize(crankshaft,
                            conrod,
                            True,
                            chrono.ChFrameD(joint_crank_conrod_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X)))
system.AddLink(rev_crank_conrod)


# --- Revolute Joint: Connecting Rod <-> Piston ---
# Joint location (piston pin) in absolute coordinates
# This is also the piston's initial position if its CoG is at the pin.
joint_conrod_piston_pos = chrono.ChVector3d(piston_x, piston_y, piston_z) # Piston pin location

rev_conrod_piston = chrono.ChLinkRevolute()
rev_conrod_piston.Initialize(conrod,
                             piston,
                             True,
                             chrono.ChFrameD(joint_conrod_piston_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X)))
system.AddLink(rev_conrod_piston)

# --- Prismatic Joint: Piston <-> Floor ---
# This constrains the piston to move along a line (e.g., X-axis relative to floor)
# Joint location (can be piston's CoG) in absolute coordinates
joint_piston_floor_pos = chrono.ChVector3d(piston_x, piston_y, piston_z)
# Prismatic joint axis (direction of sliding, here X-axis)
# The joint frame's Z-axis defines the direction of translation.
# So, we need to rotate the default frame (Z-axis along global Z)
# to make its Z-axis point along global X. This is a rotation of pi/2 around Y.
prismatic_piston_floor = chrono.ChLinkPrismatic()
prismatic_piston_floor.Initialize(piston,
                                  floor,
                                  True, # Use absolute coordinates
                                  chrono.ChFrameD(joint_piston_floor_pos, chrono.Q_from_AngY(math.pi/2)), # master frame
                                  chrono.ChFrameD(joint_piston_floor_pos, chrono.Q_from_AngY(math.pi/2))) # slave frame
system.AddLink(prismatic_piston_floor)


# ---------------------------------------------------------------------
# 5. Add Motor
# ---------------------------------------------------------------------
# The motor acts on the revolute joint between the crankshaft and the floor.
motor_function = chrono.ChFunction_Const(motor_speed_rad_s) # Constant angular speed

# ChLinkMotorRotationSpeed applies a motion to an existing ChLinkRevolute
# Note: ChLinkMotorRotation is a different class that *is* a joint itself.
# We use ChLinkMotorRotationSpeed as we already defined rev_crank_floor.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft,            # Slave body (driven by motor)
                 floor,                 # Master body (frame of reference for motor)
                 chrono.ChFrameD(joint_crank_floor_pos, chrono.Q_from_AngAxis(math.pi/2, chrono.VECT_X))) # Motor frame in abs coords
motor.SetSpeedFunction(motor_function)
system.AddLink(motor)


# ---------------------------------------------------------------------
# 6. Initialize Irrlicht Visualization
# ---------------------------------------------------------------------
myapplication = irr.ChIrrApp(system, "Crank-Slider Mechanism", irr.dimension2du(1024, 768))
myapplication.SetTimestep(system.GetTimestep()) # Synchronize Irrlicht timestep with physics
myapplication.SetTryRealtime(True)

# --- Add typical Irrlicht items ---
myapplication.AddTypicalSky(chrono.GetChronoDataFile('skybox/'))
# myapplication.AddTypicalLights() # Default lights
# Let's add a custom light
myapplication.GetSceneManager().addLight(
    None, # Parent node
    chrono.ChVector3d(30, 100, 30), # Position
    chrono.ChColor(0.8, 0.8, 0.8),  # Color (bright white)
    300.0 # Radius
)
myapplication.GetSceneManager().addLight(
    None, # Parent node
    chrono.ChVector3d(-30, 100, -30), # Position
    chrono.ChColor(0.6, 0.7, 0.8),  # Color (slight blue ambient)
    300.0 # Radius
)


# --- Customize Camera ---
# Default camera: myapplication.AddTypicalCamera(irr.vector3df(0.5, 0.5, 1.0), irr.vector3df(0,0,0))
camera = myapplication.GetSceneManager().getActiveCamera()
camera.setPosition(irr.vector3df(0.3, 0.4, 0.8)) # X, Y, Z
camera.setTarget(irr.vector3df(0.15, 0, 0))     # Look at point

# --- Add Chrono Logo ---
myapplication.GetGUIEnvironment().addImage(
    myapplication.GetVideoDriver().getTexture(chrono.GetChronoDataFile('logo_pychrono_alpha.png')),
    irr.position2di(10, 10) # Top-left corner
)

# --- Bind assets and prepare visualization ---
myapplication.AssetBindAll()
myapplication.AssetUpdateAll()


# ---------------------------------------------------------------------
# 7. Simulation Loop
# ---------------------------------------------------------------------
simulation_time = 10.0 # seconds
time = 0

while myapplication.GetDevice().run():
    myapplication.BeginScene(True, True, irr.SColor(255, 140, 160, 190)) # Background color
    myapplication.DrawAll()

    # Optional: Print piston position
    # print(f"Time: {system.GetChTime():.2f} s, Piston X: {piston.GetPos().x:.3f} m")

    # Advance the simulation
    myapplication.DoStep() # This also calls system.DoStepDynamics()

    myapplication.EndScene()

    time = system.GetChTime()
    if time > simulation_time:
        myapplication.GetDevice().closeDevice()

del myapplication # Important to release resources
print("Simulation finished.")