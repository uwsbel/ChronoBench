# =============================================================================
#
# Author: Your PyChrono Expert
#
# Description: This script demonstrates the simulation of an epicyclic gear
# system (also known as a planetary gear system) using PyChrono.
#
# The system consists of:
#   - A fixed central body (truss).
#   - A "sun" gear, rigidly attached to the central truss.
#   - A rotating "carrier" arm, hinged to the central truss.
#   - A "planet" gear, hinged to the carrier arm.
#
# A motor drives the carrier arm at a constant angular velocity, and a
# ChLinkGear constraint ensures the correct kinematic relationship between the
# sun and planet gears. The entire system is visualized using the
# Chrono::Irrlicht real-time engine.
#
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import math

# -----------------------------------------------------------------------------
# 1. Initialize the PyChrono Environment and Core Components
# -----------------------------------------------------------------------------

# Create the main PyChrono system
# We use ChSystemNSC, which is a good choice for systems with contacts and joints.
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0)) # Gravity is not needed for this 2D mechanism

# Set a solver for the system.
solver = chrono.ChSolverSOR()
system.SetSolver(solver)


# -----------------------------------------------------------------------------
# 2. Define System Parameters
# -----------------------------------------------------------------------------
# Gear and arm dimensions
sun_radius = 0.5         # Radius of the central "sun" gear
planet_radius = 0.25     # Radius of the orbiting "planet" gear
carrier_length = sun_radius + planet_radius # Distance from center to planet gear axis
gear_thickness = 0.1     # Thickness of the gears for visualization

# Motor speed
carrier_angular_speed = math.pi / 2.0  # rad/s (90 degrees per second)


# -----------------------------------------------------------------------------
# 3. Create the Physical Objects
# -----------------------------------------------------------------------------

# --- Fixed Truss (Ground) ---
# This is the central, non-moving part of the system.
truss = chrono.ChBody()
truss.SetBodyFixed(True)
truss.SetName("TRUSS")
system.Add(truss)

# --- Carrier Arm ---
# This arm rotates around the central truss and carries the planet gear.
carrier_arm = chrono.ChBody()
carrier_arm.SetName("CARRIER_ARM")
system.Add(carrier_arm)
# Set its initial position. The arm will be along the X-axis.
carrier_arm.SetPos(chrono.ChVector3d(carrier_length / 2.0, 0, 0))
carrier_arm.SetMass(1.0)
carrier_arm.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))

# --- Sun Gear ---
# The central gear, fixed to the truss.
sun_gear = chrono.ChBody()
sun_gear.SetName("SUN_GEAR")
system.Add(sun_gear)
sun_gear.SetPos(chrono.ChVector3d(0, 0, 0))
sun_gear.SetMass(2.0)
sun_gear.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))

# --- Planet Gear ---
# The outer gear that orbits the sun gear.
planet_gear = chrono.ChBody()
planet_gear.SetName("PLANET_GEAR")
system.Add(planet_gear)
# Position it at the end of the carrier arm.
planet_gear.SetPos(chrono.ChVector3d(carrier_length, 0, 0))
planet_gear.SetMass(0.5)
planet_gear.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))


# -----------------------------------------------------------------------------
# 4. Add Joints and Constraints
# -----------------------------------------------------------------------------

# --- Revolute Joint: Carrier Arm to Truss ---
# This joint allows the carrier arm to rotate around the central truss.
# The axis of rotation is the Y-axis.
revolute_carrier_truss = chrono.ChLinkLockRevolute()
revolute_carrier_truss.Initialize(
    truss,
    carrier_arm,
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(math.pi/2)) # Joint frame at origin, rotating along Y
)
system.Add(revolute_carrier_truss)

# --- Lock Joint: Sun Gear to Truss ---
# This joint rigidly fixes the sun gear to the central truss.
lock_sun_truss = chrono.ChLinkLockLock()
lock_sun_truss.Initialize(truss, sun_gear, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
system.Add(lock_sun_truss)

# --- Revolute Joint: Planet Gear to Carrier Arm ---
# This allows the planet gear to spin freely relative to the carrier arm.
revolute_planet_carrier = chrono.ChLinkLockRevolute()
revolute_planet_carrier.Initialize(
    carrier_arm,
    planet_gear,
    chrono.ChFramed(chrono.ChVector3d(carrier_length, 0, 0), chrono.Q_from_AngX(math.pi/2))
)
system.Add(revolute_planet_carrier)

# --- Gear Constraint: Sun Gear to Planet Gear ---
# This is the core constraint that defines the gear meshing relationship.
# It enforces a kinematic relation between the two gears' rotations.
gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(sun_gear, planet_gear, chrono.ChFramed(chrono.ChVector3d(0,0,0)))
# The ratio is defined as (radius of slave gear) / (radius of master gear)
gear_constraint.SetRatio(sun_radius / planet_radius)
# The master is sun_gear, the slave is planet_gear. The constraint is on Y-rotation.
gear_constraint.SetFrameShaft1(chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.Q_from_AngX(math.pi/2)))
gear_constraint.SetFrameShaft2(chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.Q_from_AngX(math.pi/2)))
system.Add(gear_constraint)

# --- Motor: Drive the Carrier Arm ---
# A motor is used to enforce a constant angular speed on the carrier arm's joint.
motor_function = chrono.ChFunctionConst(carrier_angular_speed)
carrier_motor = chrono.ChLinkMotorRotationSpeed()
carrier_motor.Initialize(
    truss,                  # Body 1
    carrier_arm,            # Body 2
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(math.pi/2)) # Motor frame
)
carrier_motor.SetSpeedFunction(motor_function)
system.Add(carrier_motor)


# -----------------------------------------------------------------------------
# 5. Add Visualization Assets
# -----------------------------------------------------------------------------

# Create visual materials for different parts
mat_blue = chrono.ChVisualMaterial()
mat_blue.SetDiffuseColor(chrono.ChColor(0.1, 0.1, 0.8))

mat_green = chrono.ChVisualMaterial()
mat_green.SetDiffuseColor(chrono.ChColor(0.1, 0.8, 0.1))

mat_red = chrono.ChVisualMaterial()
mat_red.SetDiffuseColor(chrono.ChColor(0.8, 0.1, 0.1))

mat_gray = chrono.ChVisualMaterial()
mat_gray.SetDiffuseColor(chrono.ChColor(0.4, 0.4, 0.4))

# Visualize the fixed truss as a central column
vis_truss = chrono.ChVisualShapeCylinder(0.1, 0.5)
truss.AddVisualShape(vis_truss, chrono.ChFramed(chrono.ChVector3d(0,-0.25,0)))
truss.GetVisualShape(0).SetMaterial(0, mat_gray)

# Visualize the carrier arm
vis_carrier = chrono.ChVisualShapeBox(carrier_length, 0.05, 0.05)
carrier_arm.AddVisualShape(vis_carrier)
carrier_arm.GetVisualShape(0).SetMaterial(0, mat_blue)

# Visualize the sun gear
vis_sun_gear = chrono.ChVisualShapeCylinder(sun_radius, gear_thickness)
sun_gear.AddVisualShape(vis_sun_gear, chrono.ChFramed(Vect=chrono.ChVector3d(0,0,0), Rot=chrono.Q_from_AngX(math.pi/2)))
sun_gear.GetVisualShape(0).SetMaterial(0, mat_red)

# Visualize the planet gear
vis_planet_gear = chrono.ChVisualShapeCylinder(planet_radius, gear_thickness)
planet_gear.AddVisualShape(vis_planet_gear, chrono.ChFramed(Vect=chrono.ChVector3d(0,0,0), Rot=chrono.Q_from_AngX(math.pi/2)))
planet_gear.GetVisualShape(0).SetMaterial(0, mat_green)


# -----------------------------------------------------------------------------
# 6. Set up the Irrlicht Visualization System
# -----------------------------------------------------------------------------

# Create the Irrlicht application
application = irr.ChIrrApp(
    system,
    "Epicyclic Gear System Simulation",
    irr.dimension2du(1280, 720)
)

# Add a camera to view the scene
application.AddCamera(pos=chrono.ChVector3d(0, 1.5, -2.5), lookat=chrono.ChVector3d(0, 0, 0))

# Add a light source
application.AddLight(pos=chrono.ChVector3d(3, 3, -3), radius=5)

# Finalize the application setup
application.AssetBindAll()
application.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 7. Run the Simulation Loop
# -----------------------------------------------------------------------------

application.SetTimestep(0.005)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    # Draw link frames for debugging/visualization
    irr.draw_chrono_plugin(system, "link_frames_csys")
    
    application.DoStep()
    application.EndScene()

print("Simulation finished.")