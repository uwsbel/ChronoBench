# =============================================================================
# PyChrono MAN 10t truck demo on rigid terrain
#
# Features:
#   - MAN 10t wheeled vehicle model
#   - Rigid terrain with texture
#   - TMEASY tire model
#   - Interactive real-time driver controls
#   - Irrlicht visualization with chase camera, skybox, lights, and logo
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


# -----------------------------------------------------------------------------
# Chrono / Vehicle initialization
# -----------------------------------------------------------------------------

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Simulation parameters
step_size = 2e-3
render_step_size = 1.0 / 50.0

# Contact method
contact_method = chrono.ChContactMethod_NSC

# Initial vehicle position
init_loc = chrono.ChVector3d(0.0, 0.0, 0.6)
init_rot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
init_pos = chrono.ChCoordsysd(init_loc, init_rot)

# Visualization options
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# Collision options
chassis_collision_type = veh.CollisionType_NONE
wheel_collision_type = veh.CollisionType_NONE


# -----------------------------------------------------------------------------
# Create the MAN 10t vehicle
# -----------------------------------------------------------------------------

truck = veh.MAN_10t()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(init_pos)

# Use TMEASY tire model
truck.SetTireType(veh.TireModelType_TMEASY)

# Optional: powertrain and driveline settings, if supported by your Chrono version
# truck.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
# truck.SetDriveType(veh.DrivelineTypeWV_AWD)

truck.Initialize()

truck.SetChassisVisualizationType(chassis_vis_type)
truck.SetSuspensionVisualizationType(suspension_vis_type)
truck.SetSteeringVisualizationType(steering_vis_type)
truck.SetWheelVisualizationType(wheel_vis_type)
truck.SetTireVisualizationType(tire_vis_type)

# Get the underlying Chrono system
system = truck.GetSystem()

# Solver settings
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

if contact_method == chrono.ChContactMethod_NSC:
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.GetSolver().AsIterative().SetMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)
else:
    system.SetSolverType(chrono.ChSolver.Type_MINRES)
    system.GetSolver().AsIterative().SetMaxIterations(150)


# -----------------------------------------------------------------------------
# Create rigid terrain
# -----------------------------------------------------------------------------

terrain = veh.RigidTerrain(system)

terrain_material = chrono.ChContactMaterialNSC()
terrain_material.SetFriction(0.9)
terrain_material.SetRestitution(0.01)

terrain_length = 300.0
terrain_width = 100.0

patch = terrain.AddPatch(
    terrain_material,
    chrono.ChCoordsysd(
        chrono.ChVector3d(0.0, 0.0, 0.0),
        chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
    ),
    terrain_length,
    terrain_width
)

patch.SetColor(chrono.ChColor(0.45, 0.55, 0.35))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

terrain.Initialize()


# -----------------------------------------------------------------------------
# Irrlicht visualization system
# -----------------------------------------------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t Truck on Rigid Terrain")
vis.SetWindowSize(1280, 720)

# Chase camera: target point relative to vehicle, distance, height
vis.SetChaseCamera(
    chrono.ChVector3d(0.0, 0.0, 1.75),
    8.0,
    2.5
)

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(truck.GetVehicle())


# -----------------------------------------------------------------------------
# Interactive driver
# -----------------------------------------------------------------------------

driver = veh.ChInteractiveDriverIRR(vis)

# Time to go from 0 to full input
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


# -----------------------------------------------------------------------------
# Simulation loop
# -----------------------------------------------------------------------------

render_steps = int(render_step_size / step_size)
step_number = 0

print("Controls:")
print("  Arrow up    : throttle")
print("  Arrow down  : brake")
print("  Arrow left  : steer left")
print("  Arrow right : steer right")
print("  ESC         : quit")

while vis.Run():
    time = system.GetChTime()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Render scene
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation
    driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1