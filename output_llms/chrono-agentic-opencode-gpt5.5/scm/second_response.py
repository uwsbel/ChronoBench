"""HMMWV on SCM deformable terrain using preset soil-parameter management.

This PyChrono SMC vehicle simulation drives an HMMWV over Bekker-Wong soft soil.
The SCM soil constants are encapsulated in a small class with named presets so
the terrain setup is centralized while the vehicle leaves visible rutting on the
deformable surface.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters keep vehicle, terrain, and recording coherent
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.001
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 60.0
TERRAIN_DELTA = 0.08
INIT_LOC = chrono.ChVector3d(-20.0, 0.0, 0.55)
INIT_ROT = chrono.QUNIT
TIRE_FAMILY = 1
SUPPORT_FAMILY = 4


# === SCM terrain parameters === class-owned presets replace direct setup literals
class SCMTerrainParameters:
    PRESETS = {
        "soft": (0.8e6, 0.0, 1.0, 0.0, 28.0, 0.012, 1.2e8, 2.0e4),
        "mid": (2.0e6, 0.0, 1.1, 0.0, 30.0, 0.010, 2.0e8, 3.0e4),
        "hard": (4.0e6, 0.0, 1.3, 0.0, 34.0, 0.008, 3.0e8, 4.5e4),
    }

    def __init__(self, preset):
        if preset not in self.PRESETS:
            raise ValueError(f"unknown SCM terrain preset: {preset}")
        self.name = preset
        self.values = self.PRESETS[preset]

    def apply_to(self, terrain):
        terrain.SetSoilParameters(*self.values)


# === Vehicle and system === wrapper creates the SMC system and rigid bodies
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystemSMC reused for terrain and logging
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: moving patch, logging, and camera target
vehicle_core = hmmwv.GetVehicle()  # cache: spindle/tire inspection and visualization attach
# bodies: chassis, suspension, steering, wheels, and tires are created by HMMWV_Full.
# joints: suspension and steering constraints are owned by the wrapper.
print("VEHICLE MASS: ", vehicle_core.GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === SCM preset class configures the deformable soil once
terrain = veh.SCMTerrain(system)
soil = SCMTerrainParameters("mid")
soil.apply_to(terrain)
terrain.AddMovingPatch(chassis, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetMeshWireframe(False)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 80, 80)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.15)
terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_DELTA)

tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

for axle in vehicle_core.GetAxles():
    for iw in range(2):
        tire = axle.m_wheels[iw].GetTire()
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire.GetRadius() + 0.04, tire.GetWidth()),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        spindle_collision = spindle.GetCollisionModel()
        spindle_collision.SetFamily(TIRE_FAMILY)
        spindle_collision.DisallowCollisionsWith(TIRE_FAMILY)
        spindle_collision.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()


# === Visualization and driver === Irrlicht vehicle view follows the HMMWV over SCM
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM terrain with parameter presets")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Review outputs === recording files are stripped before scoring


# === Main loop === synchronize driver, terrain, vehicle, and visualization in order
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()

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
except (RuntimeError, ValueError) as exc:
    print(f"simulation failed: {exc}")
    raise
finally:
    pass
