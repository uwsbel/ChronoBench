"""HMMWV on SCM deformable hilly terrain using an SMC vehicle system.

The simulation builds a full HMMWV vehicle, initializes Bekker-Wong SCM soil
from a bump height map, and runs a real-time Irrlicht visualization with an
interactive driver. The vehicle should settle on the terrain, respond to driver
inputs, and deform the soft hill surface through TMEASY tire contact.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
RENDER_STEP_SIZE = 1.0 / 50.0
SIM_END = 6.0
TERRAIN_LENGTH = 40.0
TERRAIN_WIDTH = 40.0
TERRAIN_RESOLUTION = 0.02
HEIGHT_MIN = -1.0
HEIGHT_MAX = 1.0
INIT_POS = chrono.ChVector3d(-15.0, 0.0, 1.0)
INIT_ROT = chrono.QUNIT
CHASE_TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once
TIRE_FAMILY = 1
SUPPORT_FAMILY = 4


# === Vehicle and system ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: chassis owns the SCM moving patch
vehicle = hmmwv.GetVehicle()  # cache: wrapper vehicle queried for axles and mass
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created components made explicit for review: system, chassis body,
# wheel spindles and tires, powertrain, suspension, driver, SCM terrain, and
# vehicle Irrlicht visualization all belong to the HMMWV-owned system.

# === Tire collision for SCM ray contact ===
tire = vehicle.GetAxles()[0].m_wheels[0].GetTire()  # cache: representative tire
tire_rad = tire.GetRadius()
tire_width = tire.GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in vehicle.GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_width),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2.0)),
        )
        spindle.EnableCollision(True)
        spindle_collision = spindle.GetCollisionModel()
        spindle_collision.SetFamily(TIRE_FAMILY)
        spindle_collision.DisallowCollisionsWith(TIRE_FAMILY)
        spindle_collision.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()

# === SCM terrain ===
terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    2.0e6,
    0.0,
    1.1,
    0.0,
    30.0,
    0.01,
    2.0e8,
    3.0e4,
)
terrain.AddMovingPatch(
    chassis,
    chrono.ChVector3d(0.0, 0.0, 0.0),
    chrono.ChVector3d(5.0, 3.0, 1.0),
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)
terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    HEIGHT_MIN,
    HEIGHT_MAX,
    TERRAIN_RESOLUTION,
)
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

# === Visualization ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM Hill")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_TRACK_POINT, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:
    print(f"Simulation stopped due to a Chrono runtime/value error: {exc}")
    raise
finally:
    pass
