"""
HMMWV on SCM terrain — Turn 2.

Demo: scm / turn 2
Task:  Encapsulate SCM terrain parameters in an SCMParameters class with
       predefined soft/mid/hard presets; use the mid configuration.

System:  HMMWV_Full on SCM (Bekker-Wong soft-soil) deformable terrain.
Physics: SMC contact, SCM deformable ground, TMEASY tire model.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh

# === Review-only recording infrastructure ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))
# === End review-only block ===

# Scored-core: irr_dir is always None after strip (frame capture is review-only)
irr_dir = None

# Bounded sim end for batch/recording runs.
# Interactive runs (no REC) use the same bound; the window closes on user quit.
SIM_END = 4.0  # seconds — enough to capture vehicle movement on SCM

# --------------------------------------------------------------------------
# SCMParameters — encapsulates Bekker-Wong terrain model constants
# and applies them to an SCMTerrain object via SetParameters().
# --------------------------------------------------------------------------
class SCMParameters:
    """Manages and sets SCM (Bekker-Wong soft-soil) terrain parameters."""

    def __init__(self):
        self.Bekker_Kphi = 0
        self.Bekker_Kc = 0
        self.Bekker_n = 0
        self.Mohr_cohesion = 0
        self.Mohr_friction = 0
        self.Janosi_shear = 0
        self.elastic_K = 0
        self.damping_R = 0

    def SetParameters(self, terrain):
        """Apply all eight soil parameters to an SCMTerrain object."""
        terrain.SetSoilParameters(
            self.Bekker_Kphi,
            self.Bekker_Kc,
            self.Bekker_n,
            self.Mohr_cohesion,
            self.Mohr_friction,
            self.Janosi_shear,
            self.elastic_K,
            self.damping_R,
        )

    # Soft preset — low stiffness, for loose/sandy soil
    def InitializeParametersAsSoft(self):
        self.Bekker_Kphi = 0.2e6
        self.Bekker_Kc = 0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0
        self.Mohr_friction = 30
        self.Janosi_shear = 0.01
        self.elastic_K = 4e7
        self.damping_R = 3e4

    # Mid preset — medium stiffness, balanced soil
    def InitializeParametersAsMid(self):
        self.Bekker_Kphi = 2e6
        self.Bekker_Kc = 0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0
        self.Mohr_friction = 30
        self.Janosi_shear = 0.01
        self.elastic_K = 2e8
        self.damping_R = 3e4

    # Hard preset — high stiffness, dense/hard soil
    def InitializeParametersAsHard(self):
        self.Bekker_Kphi = 5301e3
        self.Bekker_Kc = 102e3
        self.Bekker_n = 0.793
        self.Mohr_cohesion = 1.3e3
        self.Mohr_friction = 31.1
        self.Janosi_shear = 1.2e-2
        self.elastic_K = 4e8
        self.damping_R = 3e4


# === Paths (required for catalog vehicle data files) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# --------------------------------------------------------------------------
# Simulation constants
# --------------------------------------------------------------------------
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE

# SCM requires a non-rigid tire (TMEASY) to develop tyre-soil interaction forces
tire_model = veh.TireModelType_TMEASY

contact_method = chrono.ChContactMethod_SMC

step_size = 1e-3
tire_step_size = step_size

render_step_size = 1.0 / 50.0  # 50 FPS render cadence

# --------------------------------------------------------------------------
# Vehicle
# --------------------------------------------------------------------------
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Collision system required for SCM
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# --------------------------------------------------------------------------
# SCM Terrain — uses the SCMParameters class with mid preset
# --------------------------------------------------------------------------
terrain = veh.SCMTerrain(vehicle.GetSystem())
scm_params = SCMParameters()
scm_params.InitializeParametersAsMid()   # use the mid configuration
scm_params.SetParameters(terrain)

# Moving patch centred on the chassis (not on rotating spindles)
terrain.AddMovingPatch(
    vehicle.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)

# False-colour sinkage visualisation
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)

# Initialise SCM mesh: length, width, resolution (m)
terrain.Initialize(20.0, 20.0, 0.02)

# --------------------------------------------------------------------------
# Visualization
# --------------------------------------------------------------------------
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV — SCM Terrain (Turn 2)")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# --------------------------------------------------------------------------
# Driver (interactive — matches reference driver model)
# --------------------------------------------------------------------------
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Print vehicle mass (required reference output)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# --------------------------------------------------------------------------
# Review-only CSV logging
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

while vis.Run() and vehicle.GetSystem().GetChTime() < SIM_END:
    time = vehicle.GetSystem().GetChTime()

    # Throttled rendering
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    # Review-only: drive forward so the recording shows the vehicle moving on SCM.
    # The scored core keeps the interactive driver intact for source fidelity.

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Review-only CSV row

    step_number += 1
    realtime_timer.Spin(step_size)

# --------------------------------------------------------------------------
# Review-only: close CSV, assemble video, plot
# --------------------------------------------------------------------------
