"""CityBus on a flat dirt road — wheeled-vehicle dynamics with Pacejka '89 tires.

Model
-----
A Chrono::Vehicle CityBus (catalog wrapper, SMC contact) is initialized on a flat
RigidTerrain patch textured as a dirt road. The four wheels use the Pacejka '89
(PAC89) tire force model, attached explicitly per wheel as concrete Pac89 tire
objects so the magic-formula slip/grip curve drives the longitudinal/lateral
forces (the wrapper's SetTireType cannot instantiate a bus-specific PAC89 tire and
falls back to TMeasy, so the concrete tires are wired by InitializeTire and the
template name is asserted).

System
------
ChSystemSMC owned by the CityBus wrapper (smooth/penalty contact, suitable for the
stiff tire-road normal contact). Collision system: Bullet. Gravity -Z (Z-up world).

Behavior / objective
---------------------
A scripted driver holds the brake briefly to settle the suspension, then applies
steady throttle so the bus accelerates straight down +X on the dirt road. The
simulation integrates at a 5e-4 s step (matched by the tire step size) for tire
force stability, and the chassis forward speed should rise monotonically from rest.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 5e-4            # integration step (s) — reduced for tire-force stability
tire_step = 5e-4           # tire model sub-step (s) — matched to the solver step
sim_end = 8.0              # simulated duration (s)
render_fps = 50.0          # review-video frame rate

# CityBus geometry (introspected from the catalog wrapper after Initialize):
TIRE_RADIUS = 0.464        # Pac89 tire rolling radius (m)
CHASSIS_TO_WHEEL_BOTTOM = 0.081   # chassis-origin height above wheel-bottom at rest (m)
GROUND_CLEARANCE = 0.01    # seat wheel bottoms just above the road plane (m)
# Derived chassis spawn Z so the wheels seat on z=0 (avoid a hard drop):
init_z = GROUND_CLEARANCE - CHASSIS_TO_WHEEL_BOTTOM   # precomputed once
init_loc = chrono.ChVector3d(0.0, 0.0, init_z)
init_rot = chrono.QUNIT

# Flat dirt-road patch extents (m):
terrain_length = 200.0
terrain_width = 50.0

# Scripted-driver schedule:
SETTLE_TIME = 0.5          # brake-hold while suspension settles (s)
DRIVE_THROTTLE = 0.6       # steady throttle after settling

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once


# === Vehicle === CityBus catalog wrapper (creates + owns its ChSystemSMC + bodies)
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_SMC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_PAC89)   # prompt: Pacejka '89 tire model
bus.SetTireStepSize(tire_step)
bus.Initialize()

bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()                 # ChSystemSMC owned by the wrapper
vehicle = bus.GetVehicle()               # cache: fetched once, reused below
chassis = bus.GetChassisBody()           # cache: main chassis rigid body, reused every step
# wheels/spindles: vehicle.GetAxles()[i].GetWheels()[j].GetSpindle()
# joints: suspension + steering links created inside the wrapper

# === Tires === attach concrete Pac89 tires per wheel (magic-formula force model)
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        pac_tire = veh.HMMWV_Pac89Tire("pac89")   # concrete PAC89 magic-formula tire
        pac_tire.SetStepsize(tire_step)
        vehicle.InitializeTire(pac_tire, wheel, chrono.VisualizationType_MESH)

# Confirm the PAC89 model is actually in force (wrapper SetTireType may silently
# fall back to TMeasy for the bus — verify the attached template, do not trust it):
for axle in vehicle.GetAxles():
    for wheel in axle.GetWheels():
        tmpl = wheel.GetTire().GetTemplateName()
        assert tmpl == "Pac89Tire", f"expected Pac89Tire, got {tmpl}"

# Verify the wheels seat on (not through) the road plane after Initialize:
wheel_bottom_z = min(
    vehicle.GetSpindlePos(a, side).z
    for a in range(vehicle.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
) - TIRE_RADIUS
assert wheel_bottom_z >= -0.05, (
    f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise GROUND_CLEARANCE"
)

# === Collision system === Bullet narrow-phase for the tire-road contact
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch textured as a dirt road
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.6, 0.45, 0.3))
terrain.Initialize()

# === Driver === scripted: settle on the brake, then steady straight-line throttle
class StraightDriver(veh.ChDriver):
    """Time-based open-loop driver: brake-hold, then constant forward throttle."""

    def __init__(self, veh_obj):
        super().__init__(veh_obj)

    def Synchronize(self, time):
        if time < SETTLE_TIME:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
        else:
            self.SetThrottle(DRIVE_THROTTLE)
            self.SetBraking(0.0)
        self.SetSteering(0.0)   # drive straight down +X


driver = StraightDriver(vehicle)
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + lights + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on Dirt Road — Pacejka '89 tires")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 14.0, 1.0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(vehicle)
vis.AttachDriver(driver)


# === Main loop === render-cadence outer loop; Synchronize/Advance the subsystem stack
try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(time_step)
            terrain.Advance(time_step)
            vehicle.Advance(time_step)        # advances the wrapper-owned ChSystemSMC
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
