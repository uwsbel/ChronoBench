import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# SCMParams: encapsulates Bekker-Wong soil parameters with predefined presets
class SCMParams:
    def __init__(self, preset="mid"):
        if preset == "soft":
            self.Bekker_Kphi = 2e6       # frictional modulus (Pa)
            self.Bekker_Kc = 0           # cohesive modulus
            self.Bekker_n = 1.1          # exponent (1.0 soft)
            self.Mohr_cohesion = 0       # cohesive limit (Pa)
            self.Mohr_friction = 20      # friction angle (deg)
            self.Janosi_shear = 0.01     # shear coefficient (m)
            self.elastic_K = 2e8         # elastic stiffness (Pa/m)
            self.damping_R = 3e4         # vertical damping (Pa s/m)
        elif preset == "mid":
            self.Bekker_Kphi = 2e6       # frictional modulus (Pa)
            self.Bekker_Kc = 0           # cohesive modulus
            self.Bekker_n = 1.1          # exponent
            self.Mohr_cohesion = 0       # cohesive limit (Pa)
            self.Mohr_friction = 30      # friction angle (deg)
            self.Janosi_shear = 0.01     # shear coefficient (m)
            self.elastic_K = 2e8         # elastic stiffness (Pa/m)
            self.damping_R = 3e4         # vertical damping (Pa s/m)
        elif preset == "hard":
            self.Bekker_Kphi = 5e7       # frictional modulus (Pa) — stiff soil
            self.Bekker_Kc = 0           # cohesive modulus
            self.Bekker_n = 1.1          # exponent (harder soil)
            self.Mohr_cohesion = 0       # cohesive limit (Pa)
            self.Mohr_friction = 40      # friction angle (deg) — high friction
            self.Janosi_shear = 0.01     # shear coefficient (m)
            self.elastic_K = 2e8         # elastic stiffness (Pa/m)
            self.damping_R = 3e4         # vertical damping (Pa s/m)
        else:
            raise ValueError(f"Unknown SCMParams preset: {preset}")

    def apply(self, terrain):
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

# --- vehicle and terrain setup ---
chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # bundled data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # vehicle data path

step_size = 1e-3                                                      # physics time step (s)
sim_end = 10.0                                                        # simulation duration (s)
render_fps = 50                                                       # rendering frequency (Hz)
render_every = max(1, round(1.0 / (render_fps * step_size)))          # render cadence (untagged)

terrainLength = 100.0                                                 # terrain X extent (m)
terrainWidth = 100.0                                                  # terrain Y extent (m)

init_loc = chrono.ChVector3d(-5, 0, 0.6)                             # vehicle initial position
init_rot = chrono.QUNIT                                               # vehicle initial orientation

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                   # SMC for deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                         # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY required for SCM
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()
system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required before SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # truth's vehicle mass banner

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# --- SCM terrain with encapsulated parameters ---
terrain = veh.SCMTerrain(system)

soil_params = SCMParams("mid")                                       # use "mid" preset
soil_params.apply(terrain)                                           # apply parameters to terrain

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)         # sinkage color overlay

terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                                      # local OOBB centre offset
    chrono.ChVector3d(5, 3, 1),                                      # OOBB dims (truth: 5,3,1)
)
terrain.Initialize(terrainLength, terrainWidth, 0.1)                 # 0.1 m grid resolution
terrain.SetMeshWireframe(False)
terrain.SetTexture(
    veh.GetDataFile("terrain/textures/dirt.jpg"),
    80, 80,
)

# --- tire collision cylinders (REQUIRED for TMEASY on SCM) ---
tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()
tire_mat = chrono.ChContactMaterialSMC()
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)

TIRE_FAMILY = 1
SUPPORT_FAMILY = 4

for axle in hmmwv.GetVehicle().GetAxles():
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()
        spindle.AddCollisionShape(
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)
        sp_cm = spindle.GetCollisionModel()
        sp_cm.SetFamily(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)
        sp_cm.DisallowCollisionsWith(SUPPORT_FAMILY)

system.GetCollisionSystem().BindAll()                                 # rebuild after adding shapes

# --- Irrlicht visualization (vehicle-specific) ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV SCM Terrain — Parameterized Soil")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# --- interactive driver (truth-pattern for catalog vehicles) ---
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                                   # time to reach max steering
throttle_time = 1.0                                                   # time to reach max throttle
braking_time = 0.3                                                    # time to reach max braking
driver.SetSteeringDelta(render_fps * step_size / steering_time)
driver.SetThrottleDelta(render_fps * step_size / throttle_time)
driver.SetBrakingDelta(render_fps * step_size / braking_time)
driver.Initialize()


# --- simulation loop ---
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    if step_number % render_every == 0:                               # throttled render
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    if time >= sim_end:
        break
    realtime_timer.Spin(step_size)
