# =============================================================================
# BMW E90 Sedan on rigid terrain with TMeasy tires and interactive Irrlicht driver
#
# Controls are handled by ChInteractiveDriverIRR:
#   - steering, throttle, and braking are controlled interactively from keyboard.
#
# Requirements:
#   - PyChrono built with Vehicle and Irrlicht modules.
#   - Chrono data directory available through CHRONO_DATA_DIR or configured below.
# =============================================================================

import os
import math

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


# -----------------------------------------------------------------------------
# Data paths
# -----------------------------------------------------------------------------
chrono_data_dir = os.environ.get("CHRONO_DATA_DIR", chrono.GetChronoDataPath())
if chrono_data_dir and not chrono_data_dir.endswith(("/", "\\")):
    chrono_data_dir += "/"

chrono.SetChronoDataPath(chrono_data_dir)
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
contact_method = chrono.ChContactMethod_NSC

step_size = 1e-3
tire_step_size = 1e-3
render_step_size = 1.0 / 60.0
t_end = 1000.0

# Initial vehicle pose
init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.QUNIT

# Terrain parameters
terrain_length = 300.0
terrain_width = 100.0
terrain_friction = 0.9
terrain_restitution = 0.01

terrain_texture = veh.GetDataFile("terrain/textures/tile4.jpg")
terrain_texture_scale_x = 80
terrain_texture_scale_y = 80

# Optional visual logo/decal placed on top of the terrain
show_terrain_logo = True
terrain_logo_texture = chrono.GetChronoDataFile("logo_pychrono_alpha.png")
terrain_logo_position = chrono.ChVector3d(12.0, -8.0, 0.012)
terrain_logo_size_x = 8.0
terrain_logo_size_y = 2.5

# Visualization configuration
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Chassis collision can be changed to PRIMITIVES or MESH if supported by the model.
chassis_collision_type = veh.CollisionType_NONE

# Interactive driver response times
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3


# -----------------------------------------------------------------------------
# Helper for material creation
# -----------------------------------------------------------------------------
def create_terrain_material(method):
    if method == chrono.ChContactMethod_SMC:
        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(terrain_friction)
        mat.SetRestitution(terrain_restitution)
        mat.SetYoungModulus(2.0e7)
        mat.SetPoissonRatio(0.3)
        return mat

    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(terrain_friction)
    mat.SetRestitution(terrain_restitution)
    return mat


# -----------------------------------------------------------------------------
# Create BMW E90 sedan
# -----------------------------------------------------------------------------
if not hasattr(veh, "BMW_E90"):
    raise RuntimeError(
        "This PyChrono installation does not expose veh.BMW_E90. "
        "Make sure the Chrono Vehicle module and vehicle data are installed."
    )

bmw = veh.BMW_E90()

bmw.SetContactMethod(contact_method)
bmw.SetChassisCollisionType(chassis_collision_type)
bmw.SetChassisFixed(False)
bmw.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))

# Configure powertrain/driveline if the installed BMW wrapper exposes these APIs.
if hasattr(bmw, "SetDriveType") and hasattr(veh, "DrivelineTypeWV_RWD"):
    bmw.SetDriveType(veh.DrivelineTypeWV_RWD)

if hasattr(bmw, "SetEngineType") and hasattr(veh, "EngineModelType_SHAFTS"):
    bmw.SetEngineType(veh.EngineModelType_SHAFTS)

if hasattr(bmw, "SetTransmissionType") and hasattr(veh, "TransmissionModelType_AUTOMATIC_SHAFTS"):
    bmw.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)

# Use TMeasy tires.
bmw.SetTireType(veh.TireModelType_TMEASY)
bmw.SetTireStepSize(tire_step_size)

bmw.Initialize()

# Vehicle visualization settings.
bmw.SetChassisVisualizationType(chassis_vis_type)
bmw.SetSuspensionVisualizationType(suspension_vis_type)
bmw.SetSteeringVisualizationType(steering_vis_type)
bmw.SetWheelVisualizationType(wheel_vis_type)
bmw.SetTireVisualizationType(tire_vis_type)

system = bmw.GetSystem()


# -----------------------------------------------------------------------------
# Create rigid terrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(system)

terrain_mat = create_terrain_material(contact_method)
patch = terrain.AddPatch(
    terrain_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width
)

patch.SetTexture(
    terrain_texture,
    terrain_texture_scale_x,
    terrain_texture_scale_y
)

patch.SetColor(chrono.ChColor(0.8, 0.8, 0.75))
terrain.Initialize()


# -----------------------------------------------------------------------------
# Add an optional visual logo/decal on the terrain surface
# -----------------------------------------------------------------------------
if show_terrain_logo:
    logo_body = chrono.ChBody()
    logo_body.SetFixed(True)
    logo_body.EnableCollision(False)
    logo_body.SetPos(terrain_logo_position)

    logo_shape = chrono.ChVisualShapeBox(
        terrain_logo_size_x,
        terrain_logo_size_y,
        0.01
    )
    logo_shape.SetTexture(terrain_logo_texture)

    logo_body.AddVisualShape(logo_shape)
    system.Add(logo_body)


# -----------------------------------------------------------------------------
# Irrlicht visualization system
# -----------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("PyChrono BMW E90 Sedan - Rigid Terrain / TMeasy Tires")
vis.SetWindowSize(1280, 720)

# Chase camera target point in the vehicle frame, distance, and height.
vis.SetChaseCamera(
    chrono.ChVector3d(0.0, 0.0, 1.2),
    7.0,
    0.6
)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(bmw.GetVehicle())


# -----------------------------------------------------------------------------
# Interactive driver
# -----------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = math.ceil(render_step_size / step_size)

step_number = 0

while vis.Run():
    time = system.GetChTime()

    if time >= t_end:
        break

    # Get driver inputs.
    driver_inputs = driver.GetInputs()

    # Synchronize all modules.
    driver.Synchronize(time)
    terrain.Synchronize(time)
    bmw.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Render.
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Advance simulation.
    driver.Advance(step_size)
    terrain.Advance(step_size)
    bmw.Advance(step_size)
    vis.Advance(step_size)

    # Real-time pacing.
    realtime_timer.Spin(step_size)

    step_number += 1