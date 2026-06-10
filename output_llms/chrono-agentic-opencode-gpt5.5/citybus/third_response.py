"""CityBus rigid-terrain simulation using NSC contact.

The script builds a PyChrono CityBus on a flat rigid road, requests the
Pacejka 89 tire model, uses a 5e-4 s vehicle and tire step size, and applies a
dirt terrain texture. The bus is visualized with Irrlicht and driven through the
standard vehicle subsystem synchronization stack.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
# Vehicle and terrain parameters are explicit so the requested edits are visible.
step_size = 5e-4
tire_step_size = 5e-4
sim_end = 6.0
render_fps = 50.0
render_step_size = 1.0 / render_fps
render_every = max(1, round(1.0 / (render_fps * step_size)))  # precomputed once
terrain_length = 200.0
terrain_width = 20.0
terrain_texture_u = 120.0
terrain_texture_v = 12.0
init_loc = chrono.ChVector3d(0.0, 0.0, -0.02)
init_rot = chrono.QUNIT

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


# === Vehicle ===
# The CityBus wrapper owns its ChSystem; configure it before initialization.
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_PAC89)  # prompt: Pacejka tire model, 89 version
bus.SetTireStepSize(tire_step_size)
bus.Initialize()

system = bus.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = bus.GetVehicle()  # cache: full vehicle handle reused for mass, wheels, visualization
chassis = bus.GetChassisBody()  # cache: chassis body reused for logging and camera chase point
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created components are the CityBus system, chassis, axles, tires,
# terrain, Irrlicht visualizer, and driver; all are stepped below.
tire_radius = vehicle.GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # cache: spawn check radius
spindle_positions = []
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - tire_radius
assert wheel_bottom_z >= -0.05, (
    f"CityBus starts below terrain: wheel bottom z={wheel_bottom_z:.3f}"
)

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain ===
# A flat rigid road uses NSC material and the requested dirt texture.
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), terrain_texture_u, terrain_texture_v)
patch.SetColor(chrono.ChColor(0.45, 0.36, 0.25))
terrain.Initialize()


# === Visualization And Driver ===
# Vehicle Irrlicht visualization follows the catalog demo order.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus Pacejka 89 on Dirt Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 25.0, 3.0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Review Data Files ===


# === Main Loop ===
# The loop renders at a fixed cadence and advances all vehicle subsystems in order.
frame = 0
step_number = 0
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
            bus.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(step_size)
            terrain.Advance(step_size)
            bus.Advance(step_size)
            vis.Advance(step_size)

            step_number += 1
            realtime_timer.Spin(step_size)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError, OSError) as exc:
    traceback.print_exc()
    raise
finally:
    pass


# === Post Processing ===
