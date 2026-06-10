"""MAN 5t truck on a rigid height-map hill terrain.

This NSC vehicle simulation uses the PyChrono MAN_5t catalog wrapper, a rigid
height-map terrain patch textured with grass, and an Irrlicht interactive driver
view. The truck starts at (-20, 0, 1.5) and is expected to drive across the
rolling rigid hills while maintaining contact through the vehicle terrain stack.
"""

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named setup values keep the vehicle and terrain configuration explicit
step_size = 2e-3
sim_end = 6.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once
terrain_length = 80.0
terrain_width = 40.0
height_min = -1.0
height_max = 1.0
init_pos = chrono.ChVector3d(-20.0, 0.0, 1.5)
init_rot = chrono.QUNIT


# === Vehicle setup === catalog wrapper owns the ChSystem and all vehicle bodies
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.MAN_5t()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: main chassis body sampled every step
vehicle_core = vehicle.GetVehicle()  # cache: wrapper vehicle interface reused for mass and spindle checks
# wrapper-created bodies: chassis, axles, wheels, tires; wrapper-created joints: suspension, steering, driveline
print("VEHICLE MASS: ", vehicle_core.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid height-map hills with prompt-requested grass texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    terrain_length,
    terrain_width,
    height_min,
    height_max,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 12.0, 12.0)
patch.SetColor(chrono.ChColor(0.6, 0.8, 0.45))
terrain.Initialize()

spindle_positions = []
for axle_index in range(vehicle_core.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle_core.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - 0.6
assert wheel_bottom_z >= height_min - 0.1, (
    f"vehicle starts below terrain height range: wheel bottom z={wheel_bottom_z:.3f}, "
    f"minimum terrain z={height_min:.3f}"
)


# === Visualization === vehicle-specific Irrlicht window follows the truck over the hills
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t on Rigid Hills")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta((1.0 / render_fps) / steering_time)
driver.SetThrottleDelta((1.0 / render_fps) / throttle_time)
driver.SetBrakingDelta((1.0 / render_fps) / braking_time)
driver.Initialize()


# === Main loop === synchronize the full driver, terrain, vehicle, and visualization stack
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(step_size)
            terrain.Advance(step_size)
            vehicle.Advance(step_size)
            vis.Advance(step_size)

            if system.GetChTime() >= sim_end:
                break
            realtime_timer.Spin(step_size)
except (OSError, IOError) as exc:
    # guard: OS-level failures from renderer or asset access
    raise RuntimeError(f"failed to access simulation resources: {exc}") from exc
except (RuntimeError, ValueError) as exc:
    # guard: Chrono solver or bad-state errors should surface with context
    raise RuntimeError(f"MAN terrain simulation failed: {exc}") from exc
finally:
    pass
