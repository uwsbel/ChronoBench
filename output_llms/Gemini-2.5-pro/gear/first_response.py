import pychrono as chrono
import pychrono.irrlicht as irr
import math

# 1. Initialize PyChrono environment and core components
chrono.SetChronoDataPath('./chrono_data/') # Set path to Chrono data files (textures, etc.)

# Create a Chrono system
system = chrono.ChSystemNSC() # Using Non-Smooth Contact (NSC) solver

# Set gravity (though not strictly necessary for this kinematic demo if well-constrained)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81)) # Z-axis up for convention

# --- Parameters ---
sun_radius = 0.5  # meters
planet_radius = 0.25 # meters
gear_thickness = 0.1 # meters
carrier_arm_length = sun_radius + planet_radius # Distance from center to planet gear axis
carrier_arm_visual_radius = 0.05 # For visualization of the arm bar
motor_speed_rad_s = math.pi / 2  # 0.5 rad/s, adjust as needed (approx 1 rotation every 4s)

# Material properties (for visualization)
mat_grey = chrono.ChVisualMaterial()
mat_grey.SetDiffuseColor(chrono.ChColor(0.3, 0.3, 0.3))
mat_grey.SetSpecularColor(chrono.ChColor(0.5,0.5,0.5))

mat_blue = chrono.ChVisualMaterial()
mat_blue.SetDiffuseColor(chrono.ChColor(0.2, 0.2, 0.8))
mat_blue.SetSpecularColor(chrono.ChColor(0.5,0.5,0.8))

mat_green = chrono.ChVisualMaterial()
mat_green.SetDiffuseColor(chrono.ChColor(0.2, 0.8, 0.2))
mat_green.SetSpecularColor(chrono.ChColor(0.5,0.8,0.5))

mat_red = chrono.ChVisualMaterial()
mat_red.SetDiffuseColor(chrono.ChColor(0.8, 0.2, 0.2))
mat_red.SetSpecularColor(chrono.ChColor(0.8,0.5,0.5))


# --- Helper function for creating cylindrical bodies ---
def create_gear_body(name, radius, thickness, position, material, is_fixed=False):
    body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, # Axis of cylinder
                                     radius, thickness, 1000, # radius, height, density
                                     True, True) # enable collision and visualization
    body.SetName(name)
    body.SetPos(position)
    # Cylinders are created along Y, rotate them to be along Z for our setup
    body.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
    if is_fixed:
        body.SetFixed(True)
    body.GetVisualShape(0).SetMaterial(0, material)
    system.Add(body)
    return body

# 2. Add required physical systems and objects

# --- Fixed Truss (Ground) ---
# We'll use an invisible fixed body as the reference frame.
# Or, for visualization, a small central truss.
truss = chrono.ChBodyEasyBox(0.1, 0.1, 0.2, 1000, True, False) # Small box
truss.SetName("Truss (Ground)")
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
truss.GetVisualShape(0).SetMaterial(0, mat_grey)
system.Add(truss)


# --- Sun Gear ---
sun_gear_pos = chrono.ChVector3d(0, 0, 0)
sun_gear = create_gear_body("SunGear", sun_radius, gear_thickness, sun_gear_pos, mat_blue)

# --- Carrier Arm ---
# For simplicity, the carrier arm will be a rigid body.
# We'll visualize it as a bar.
carrier_arm_body = chrono.ChBodyEasyBox(carrier_arm_length * 1.5, # length (along X)
                                     carrier_arm_visual_radius * 2, # width (along Y)
                                     carrier_arm_visual_radius * 2, # height (along Z)
                                     1000, True, True)
carrier_arm_body.SetName("CarrierArm")
carrier_arm_body.SetPos(chrono.ChVector3d(carrier_arm_length * 0.75 / 2, 0, 0)) # Position its CoG
carrier_arm_body.GetVisualShape(0).SetMaterial(0, mat_green)
system.Add(carrier_arm_body)
# Note: The carrier arm's rotation axis will be at (0,0,0).
# The planet gear will be attached at carrier_arm_length from (0,0,0) along the arm.

# --- Planet Gear ---
# Initial position: assuming carrier arm starts aligned with X-axis
planet_gear_pos_initial = chrono.ChVector3d(carrier_arm_length, 0, 0)
planet_gear = create_gear_body("PlanetGear", planet_radius, gear_thickness, planet_gear_pos_initial, mat_red)


# 3. Set necessary default parameters: joints and motor

# --- Revolute Joint: Sun Gear to Truss ---
# The rotation axis for all revolute joints will be the global Z-axis.
# The ChFrameD defines the joint's position and orientation (Z-axis is the rotation axis).
z_rot_frame = chrono.QuatFromAngleY(chrono.CH_PI_2) # Align Z with revolute axis

rev_sun_truss = chrono.ChLinkRevolute()
rev_sun_truss.Initialize(sun_gear,              # body1
                         truss,                 # body2
                         True,                  # an absolute ChFrame for the joint
                         chrono.ChFrameD(sun_gear_pos, z_rot_frame)) # Joint frame
system.Add(rev_sun_truss)

# --- Revolute Joint: Carrier Arm to Truss ---
rev_carrier_truss = chrono.ChLinkRevolute()
rev_carrier_truss.Initialize(carrier_arm_body,
                             truss,
                             True,
                             chrono.ChFrameD(chrono.ChVector3d(0,0,0), z_rot_frame))
system.Add(rev_carrier_truss)

# --- Revolute Joint: Planet Gear to Carrier Arm ---
# The joint is located at the end of the carrier arm in its local frame.
# For initialization, we can use absolute coordinates based on initial setup.
# Joint position: (carrier_arm_length, 0, 0) in global if arm is along X.
# Or, define relative to carrier:
#   joint_pos_on_carrier = chrono.ChVector3d(carrier_arm_length, 0, 0)
#   joint_pos_on_planet = chrono.ChVector3d(0,0,0) # At planet's center
#   rev_planet_carrier.Initialize(planet_gear, carrier_arm_body, False,
#                                chrono.ChFrameD(joint_pos_on_planet),
#                                chrono.ChFrameD(joint_pos_on_carrier),
#                                True) # auto_align = True

rev_planet_carrier = chrono.ChLinkRevolute()
rev_planet_carrier.Initialize(planet_gear,
                              carrier_arm_body,
                              True, # use absolute frame
                              chrono.ChFrameD(planet_gear_pos_initial, z_rot_frame))
system.Add(rev_planet_carrier)


# --- Gear Constraint: Sun Gear to Planet Gear ---
# This links the rotation of the sun gear (relative to truss)
# with the rotation of the planet gear (relative to carrier arm).
link_gear_sun_planet = chrono.ChLinkGear()
# Initialize with the two revolute joints to be constrained, and a master frame.
# The master frame's Z-axis should be aligned with the rotation axes of the revolute joints.
link_gear_sun_planet.Initialize(rev_sun_truss, rev_planet_carrier, chrono.ChFrameD(chrono.ChVector3d(0,0,0), z_rot_frame))

# Transmission ratio: omega_planet_rel_carrier / omega_sun_rel_truss
# For external gears, if sun rotates CW, planet rotates CCW relative to its pivot on carrier.
# So, ratio is negative. Ratio magnitude is R_sun / R_planet.
transmission_ratio = -sun_radius / planet_radius
link_gear_sun_planet.SetTransmissionRatio(transmission_ratio)
link_gear_sun_planet.SetEnforcePhase(True) # Try to maintain initial relative angle
link_gear_sun_planet.SetPhase(0)
system.Add(link_gear_sun_planet)


# --- Motor: Drive the Sun Gear ---
motor_function = chrono.ChFunctionConst(motor_speed_rad_s) # Constant speed

# The motor acts on the sun_gear relative to the truss (same bodies as rev_sun_truss)
# It could also be ChLinkMotorRotationAngle, ChLinkMotorRotationTorque, etc.
sun_motor = chrono.ChLinkMotorRotationSpeed()
sun_motor.Initialize(sun_gear,      # Slave body (driven)
                     truss,         # Master body (reference)
                     chrono.ChFrameD(sun_gear_pos, z_rot_frame)) # Motor frame aligned with revolute joint
sun_motor.SetSpeedFunction(motor_function)
system.Add(sun_motor)


# --- Irrlicht Visualization Setup ---
myapplication = irr.ChVisualSystemIrrlicht()
myapplication.AttachSystem(system)
myapplication.SetWindowSize(1024, 768)
myapplication.SetWindowTitle('PyChrono Epicyclic Gear Demo')
myapplication.Initialize()
myapplication.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddSkyBox()
myapplication.AddTypicalLights()
# Camera position: look along -Y, Z up
myapplication.AddCamera(chrono.ChVector3d(0, -2.5 * carrier_arm_length, 0.5 * carrier_arm_length), # Camera position
                        chrono.ChVector3d(0, 0, 0))                   # Look at point

# Set a custom camera closer to the action if needed
# myapplication.GetActiveCamera().SetTarget(chrono.ChVector3d(0,0,0))
# myapplication.GetActiveCamera().SetPosition(chrono.ChVector3d(carrier_arm_length*1.5, -carrier_arm_length*1.5, carrier_arm_length*0.5))

# --- Simulation Loop ---
timestep = 0.005
system.SetTimestep(timestep)
# Optional: use a more robust solver if things get unstable, though for this it should be fine
# system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
# system.SetSolverMaxIterations(100)


while myapplication.GetDevice().run():
    myapplication.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.15)) # Dark background
    myapplication.Render()
    myapplication.DrawAll() # This is important to draw constraint visualizations if enabled

    # Optional: Print some info
    # print(f"Time: {system.GetChTime():.2f} s, Sun Angle: {rev_sun_truss.GetRelativeAngle() * 180/math.pi:.1f} deg, Carrier Angle: {rev_carrier_truss.GetRelativeAngle() * 180/math.pi:.1f} deg")

    system.DoStepDynamics(timestep)
    myapplication.EndScene()

# Proper cleanup (optional, as Python's GC will handle it, but good practice)
del myapplication
del system