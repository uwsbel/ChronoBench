"""HMMWV on SCM deformable hill terrain.

This PyChrono SMC vehicle simulation places a full HMMWV on a heightmap-based
SCM soil patch. The wrapper vehicle, TMeasy tires, interactive Irrlicht driver,
SCM terrain, and vehicle visual system are synchronized in real time so the
vehicle can climb and deform the soft hill surface.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Direct demo constants keep the vehicle setup close to the catalog SCM example.
STEP_SIZE = 1e-3
TIRE_STEP_SIZE = 1e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 40.0
TERRAIN_WIDTH = 40.0
TERRAIN_HMIN = -1.0
TERRAIN_HMAX = 1.0
TERRAIN_DELTA = 0.02
INIT_LOC = chrono.ChVector3d(-15.0, 0.0, 0.7)
INIT_ROT = chrono.QUNIT

TIRE_FAMILY = 1
SUPPORT_FAMILY = 4


# === Vehicle ===
# The HMMWV wrapper owns the SMC system, chassis, suspension, steering, and wheels.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemSMC reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: vehicle subsystem reused for mass and spindle queries
chassis = hmmwv.GetChassisBody()  # cache: chassis body anchors the SCM moving patch
print("VEHICLE MASS: ", vehicle.GetMass())

# wrapper-created bodies: chassis, suspension links, steering links, spindles, wheels, and tires
# wrapper-created joints: suspension, steering, driveline, and wheel rotational constraints

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain ===
# SCM terrain uses a hill heightmap and Bekker-Wong soil parameters.
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2e6,   # Bekker_Kphi
    0.0,   # Bekker_Kc
    1.1,   # Bekker_n
    0.0,   # Mohr cohesion
    30.0,  # Mohr friction angle
    0.01,  # Janosi shear
    2e8,   # elastic stiffness
    3e4,   # damping
)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetMeshWireframe(False)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_HMIN,
    TERRAIN_HMAX,
    TERRAIN_DELTA,
)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in vehicle.GetAxles():
    for iw in range(2):
        wheel = axle.m_wheels[iw]
        spindle = wheel.GetSpindle()
        tire = wheel.GetTire()
        tire_rad = tire.GetRadius()
        tire_width = tire.GetWidth()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_width),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        spindle_collision = spindle.GetCollisionModel()
        spindle_collision.SetFamily(TIRE_FAMILY)
        spindle_collision.DisallowCollisionsWith(TIRE_FAMILY)
        spindle_collision.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()

spindle_positions = []  # cache: post-initialize wheel positions validate terrain support
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
min_wheel_bottom = min(p.z for p in spindle_positions) - tire_rad
assert min_wheel_bottom > TERRAIN_HMIN - 0.2, "HMMWV wheels initialize below the SCM height range"


# === Visualization And Driver ===
# Vehicle-specific Irrlicht visualizer follows the catalog real-time demo shape.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Hill")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta((1.0 / RENDER_FPS) / steering_time)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / throttle_time)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / braking_time)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0


# === Main Loop ===
# Synchronize and advance the full driver, terrain, vehicle, and visual stack.
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()  # cache: consumed by vehicle and visual sync


            driver.Synchronize(time)
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid vehicle state
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
