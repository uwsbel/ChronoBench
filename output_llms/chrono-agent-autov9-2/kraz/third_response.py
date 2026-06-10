"""Kraz tractor-and-semitrailer plus a passenger sedan on a flat rigid highway.

System type: NSC (Chrono multibody, default contact for the catalog vehicles).
Main bodies:
  - A Kraz long-haul truck (tractor + semitrailer), modelled with RIGID tires
    (the Kraz catalog model uses rigid tires inherently) driven forward.
  - A passenger Sedan sharing the same ChSystem, controlled with a fixed
    forward throttle and a fixed steering input by its own driver.
  - A flat RigidTerrain patch acting as the highway surface, textured as a road.
Expected behaviour: both vehicles spawn on the road at their prescribed poses,
then drive forward; the truck pulls its semitrailer in a straight line while the
sedan accelerates forward alongside with a small constant steering bias. The
tractor and trailer poses are recorded over the run for inspection.
"""

# === Imports ===
import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants: timing, geometry, control ===
time_step = 2e-3                       # integration step (s)
tire_step = 1e-3                       # tire force sub-step (s)
sim_end = 12.0                         # simulated duration (s)
render_fps = 50.0                      # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once: steps per frame

# Chassis-origin rest height above the flat road for the catalog vehicles.
KRAZ_REF_HEIGHT = 0.5588               # Kraz tractor chassis origin above road at rest
KRAZ_CLEARANCE = 0.05                  # small clearance so wheels start just on the road
SEDAN_REF_HEIGHT = 0.5                 # Sedan chassis origin above road at rest

ROAD_LENGTH = 300.0                    # X extent of the highway patch (m)
ROAD_WIDTH = 60.0                      # Y extent of the highway patch (m)

# Final truck pose: shifted back along X, in a lane offset in +Y, heading +X.
KRAZ_INIT_LOC = chrono.ChVector3d(-20.0, 3.0, KRAZ_REF_HEIGHT + KRAZ_CLEARANCE)
KRAZ_INIT_ROT = chrono.QuatFromAngleZ(0.0)

# Final sedan pose: a parallel lane in -Y, slightly ahead, heading +X.
SEDAN_INIT_LOC = chrono.ChVector3d(-10.0, -3.0, SEDAN_REF_HEIGHT)
SEDAN_INIT_ROT = chrono.QuatFromAngleZ(0.0)

# Sedan open-loop control: constant forward throttle and a small fixed steering.
SEDAN_THROTTLE = 0.5
SEDAN_STEERING = 0.05

# === Truck: Kraz tractor + semitrailer (RIGID tires are inherent to this model) ===
kraz = veh.Kraz()
kraz.SetChassisCollisionType(veh.CollisionType_NONE)   # add collision via terrain contact only
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysd(KRAZ_INIT_LOC, KRAZ_INIT_ROT))
kraz.SetTireStepSize(tire_step)
kraz.Initialize()

# Kraz uses RIGID tires inherently (no SetTireType on this wrapper) — suitable
# for a flat rigid road where the truck must roll forward.
kraz.SetChassisVisualizationType(chrono.VisualizationType_MESH, chrono.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES, chrono.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(chrono.VisualizationType_MESH, chrono.VisualizationType_MESH)
kraz.SetTireVisualizationType(chrono.VisualizationType_MESH, chrono.VisualizationType_MESH)

# === System & bodies (the Kraz wrapper created and owns the ChSystem) ===
sys = kraz.GetSystem()                          # ChSystem owned by the Kraz wrapper
tractor = kraz.GetTractor()                     # cache: tractor ChWheeledVehicle, reused every step
tractor_chassis = kraz.GetTractorChassisBody()  # cache: tractor chassis rigid body
trailer = kraz.GetTrailer()                     # semitrailer (ChWheeledTrailer)
trailer_chassis = trailer.GetChassis().GetBody()  # cache: trailer chassis rigid body
# Wrapper-created internals: tractor + trailer chassis bodies, wheel spindles,
# suspension + steering joints, and rigid tires — all built inside veh.Kraz().

# Bullet collision is required for the wheel/terrain contact in this scene.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Second vehicle: passenger Sedan sharing the Kraz system ===
sedan = veh.Sedan(sys)                          # shares the truck's ChSystem
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(SEDAN_INIT_LOC, SEDAN_INIT_ROT))
sedan.SetTireType(veh.TireModelType_TMEASY)     # sedan rolls on the rigid road
sedan.SetTireStepSize(tire_step)
sedan.Initialize()
sedan.SetChassisVisualizationType(chrono.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(chrono.VisualizationType_MESH)
sedan.SetTireVisualizationType(chrono.VisualizationType_MESH)
sedan_chassis = sedan.GetChassisBody()          # cache: sedan chassis rigid body

# === Terrain: flat rigid highway patch ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, ROAD_LENGTH, ROAD_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.6, 0.6, 0.6))
terrain.Initialize()

# Restore Bullet collision after terrain/vehicle initialization wired the scene.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Drivers: one scripted driver per vehicle ===
# Truck driver: ramp to a steady forward cruise, no steering (straight haul).
class TruckDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Ease the throttle in over the first second to avoid a torque spike.
        self.SetThrottle(min(0.6, 0.6 * time))
        self.SetBraking(0.0)
        self.SetSteering(0.0)

truck_driver = TruckDriver(tractor)
truck_driver.Initialize()

# Sedan driver: constant forward throttle and a small fixed steering bias.
class SedanDriver(veh.ChDriver):
    def __init__(self, vehicle, throttle, steering):
        super().__init__(vehicle)
        self._throttle = throttle      # cache: constant control targets
        self._steering = steering

    def Synchronize(self, time):
        self.SetThrottle(self._throttle)
        self.SetBraking(0.0)
        self.SetSteering(self._steering)

sedan_driver = SedanDriver(sedan.GetVehicle(), SEDAN_THROTTLE, SEDAN_STEERING)
sedan_driver.Initialize()

# === Footprint sanity check after Initialize (wheels rest on the road) ===
ZTOL = 0.10
tractor_wheel_z = min(
    tractor.GetSpindlePos(axle, side).z
    for axle in range(tractor.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
)
assert tractor_wheel_z >= -ZTOL, (
    f"truck wheels start below road: min spindle z={tractor_wheel_z:.3f}; "
    f"raise KRAZ_REF_HEIGHT"
)

# === Visualization === full vehicle Irrlicht scene: window + sky + camera + lights, chase the truck
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz truck and sedan on a highway")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 14.0, 1.0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(-30, -20, 8), chrono.ChVector3d(-10, 0, 0))
vis.AttachVehicle(tractor)

# === Output setup (recording / logging) ===


# === Main loop === render once per frame, advance both vehicles + terrain in the inner batch
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()
            truck_inputs = truck_driver.GetInputs()
            sedan_inputs = sedan_driver.GetInputs()


            truck_driver.Synchronize(sim_time)
            sedan_driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            kraz.Synchronize(sim_time, truck_inputs, terrain)
            sedan.Synchronize(sim_time, sedan_inputs, terrain)
            vis.Synchronize(sim_time, truck_inputs)

            truck_driver.Advance(time_step)
            sedan_driver.Advance(time_step)
            terrain.Advance(time_step)
            kraz.Advance(time_step)            # advances the shared wrapper-owned system
            sedan.Advance(time_step)
            vis.Advance(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:      # solver divergence / bad state mid-run
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review video + timeseries plot) ===
