"""HMMWV on SCM deformable terrain with preset soil parameters.

This PyChrono 9.0 SMC vehicle scene drives an HMMWV over SCMTerrain.  The SCM
Bekker-Wong soil values are managed by SCMTerrainParameters presets so the
terrain setup is explicit, reusable, and independent from the vehicle setup.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named values keep vehicle, terrain, and recording parameters visible
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_STEPS = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 20.0
TERRAIN_DELTA = 0.08
TERRAIN_PRESET = "mid"
INIT_POS = chrono.ChVector3d(-25.0, 0.0, 0.50)  # precomputed once
INIT_ROT = chrono.QUNIT

TIRE_FAMILY = 1
CHASSIS_FAMILY = 2
SUPPORT_FAMILY = 4
WHEEL_Z_TOL = 0.20


# === Terrain parameters === class owns SCM soil presets and applies all eight fields
class SCMTerrainParameters:
    """Preset SCM parameter bundles for soft, mid, and hard soil."""

    PRESETS = {
        "soft": (0.2e6, 0.0, 1.10, 0.0, 25.0, 0.010, 4.0e7, 2.0e4),
        "mid": (2.0e6, 0.0, 1.10, 0.0, 30.0, 0.010, 2.0e8, 3.0e4),
        "hard": (4.0e6, 0.0, 1.20, 1.0e3, 35.0, 0.015, 3.0e8, 4.0e4),
    }

    def __init__(self, preset_name):
        try:
            self.values = self.PRESETS[preset_name]
        except KeyError as exc:  # invalid preset name supplied by caller
            valid = ", ".join(sorted(self.PRESETS))
            raise ValueError(f"unknown SCM terrain preset {preset_name!r}; choose {valid}") from exc
        self.preset_name = preset_name

    def apply_to(self, terrain):
        terrain.SetSoilParameters(*self.values)


def add_tire_collision_cylinders(hmmwv, system):
    """Add SCM ray-hit tire geometry for non-rigid TMEASY tires."""
    tire_mat = chrono.ChContactMaterialSMC()
    tire_mat.SetFriction(0.9)
    tire_mat.SetRestitution(0.1)

    vehicle_model = hmmwv.GetVehicle()  # cache: wrapper vehicle handle reused here
    first_tire = vehicle_model.GetAxles()[0].m_wheels[0].GetTire()  # cache: tire geometry source
    tire_radius = first_tire.GetRadius()
    tire_width = first_tire.GetWidth()

    for axle in vehicle_model.GetAxles():
        for iw in range(2):
            spindle = axle.m_wheels[iw].GetSpindle()
            spindle.AddCollisionShape(
                chrono.ChCollisionShapeCylinder(tire_mat, tire_radius + 0.04, tire_width),
                chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2.0)),
            )
            spindle.EnableCollision(True)
            spindle_cm = spindle.GetCollisionModel()
            spindle_cm.SetFamily(TIRE_FAMILY)
            spindle_cm.DisallowCollisionsWith(TIRE_FAMILY)
            spindle_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

    system.GetCollisionSystem().BindAll()
    return tire_radius


def assert_vehicle_on_scm(hmmwv, tire_radius):
    """Check that the initialized wheel bottoms are near the SCM rest plane."""
    vehicle_model = hmmwv.GetVehicle()  # cache: spindle positions are read from this handle
    spindle_positions = []
    for axle_index in range(vehicle_model.GetNumberAxles()):
        spindle_positions.append(vehicle_model.GetSpindlePos(axle_index, veh.LEFT))
        spindle_positions.append(vehicle_model.GetSpindlePos(axle_index, veh.RIGHT))
    wheel_bottom_z = min(pos.z for pos in spindle_positions) - tire_radius
    assert wheel_bottom_z >= -WHEEL_Z_TOL, (
        f"wheel bottom z={wheel_bottom_z:.3f} is too far below SCM plane; "
        "raise INIT_POS.z before initializing the HMMWV"
    )


def main():
    # === Vehicle system === HMMWV wrapper owns the SMC system used by terrain and tires
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

    system = hmmwv.GetSystem()  # cache: shared owned system for terrain and collision setup
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    vehicle_model = hmmwv.GetVehicle()  # cache: reused for visualization, mass, and driver
    chassis = hmmwv.GetChassisBody()  # cache: moving SCM patch follows the stable chassis
    print("VEHICLE MASS: ", vehicle_model.GetMass())

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    tire_radius = add_tire_collision_cylinders(hmmwv, system)
    assert_vehicle_on_scm(hmmwv, tire_radius)

    # === SCM terrain === preset class replaces direct soil parameter assignment
    terrain = veh.SCMTerrain(system)
    soil = SCMTerrainParameters(TERRAIN_PRESET)
    soil.apply_to(terrain)
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.10)
    terrain.SetMeshWireframe(False)
    terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 12.0, 12.0)
    terrain.AddMovingPatch(
        chassis,
        chrono.ChVector3d(0.0, 0.0, 0.0),
        chrono.ChVector3d(5.0, 3.0, 1.0),
    )
    terrain.Initialize(TERRAIN_LENGTH, TERRAIN_WIDTH, TERRAIN_DELTA)

    # === Visualization and driver === vehicle Irrlicht stack mirrors the catalog demo shape
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("SCM Terrain Parameter Presets")
    vis.SetWindowSize(1280, 720)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddLightDirectional()
    vis.AttachVehicle(vehicle_model)

    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(STEP_SIZE / 1.0)
    driver.SetThrottleDelta(STEP_SIZE / 1.0)
    driver.SetBrakingDelta(STEP_SIZE / 0.3)
    driver.Initialize()

    # === Review outputs === CSV and frame directories exist only for record-mode validation
    data_file = None

    # === Main loop === synchronize full driver, terrain, vehicle, and visual stacks each step
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            if step_number % RENDER_STEPS == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError, AssertionError) as exc:  # solver/API failures or invalid placement
        traceback.print_exc()
        raise
    finally:
        if data_file is not None:
            data_file.close()


if __name__ == "__main__":
    try:
        main()
    except (OSError, IOError) as exc:  # output directory or CSV file access failure
        traceback.print_exc()
        raise
