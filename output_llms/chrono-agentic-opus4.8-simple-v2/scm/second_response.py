import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# Encapsulated SCM terrain parameters: holds the 8 Bekker-Wong/Mohr soil
# coefficients and applies them to an SCMTerrain via SetSoilParameters.
class SCMParameters:
    def __init__(self):
        self.Bekker_Kphi = 0.0      # frictional modulus (Pa)
        self.Bekker_Kc = 0.0        # cohesive modulus (Pa)
        self.Bekker_n = 0.0         # sinkage exponent
        self.Mohr_cohesion = 0.0    # cohesive limit (Pa)
        self.Mohr_friction = 0.0    # friction angle (deg)
        self.Janosi_shear = 0.0     # shear deformation modulus (m)
        self.elastic_K = 0.0        # elastic stiffness (Pa/m)
        self.damping_R = 0.0        # vertical damping (Pa.s/m)

    def SetParameters(self, terrain):                               # push the 8 coefficients into the terrain
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

    def InitializeParametersAsSoft(self):                          # loose, easily-deformed soil
        self.Bekker_Kphi = 0.2e6
        self.Bekker_Kc = 0.0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0.0
        self.Mohr_friction = 30.0
        self.Janosi_shear = 0.01
        self.elastic_K = 4e7
        self.damping_R = 3e4

    def InitializeParametersAsMid(self):                           # intermediate stiffness soil
        self.Bekker_Kphi = 0.2e6
        self.Bekker_Kc = 0.0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0.0
        self.Mohr_friction = 30.0
        self.Janosi_shear = 0.01
        self.elastic_K = 2e8
        self.damping_R = 3e4

    def InitializeParametersAsHard(self):                          # compacted, stiff soil
        self.Bekker_Kphi = 5301e3
        self.Bekker_Kc = 102e3
        self.Bekker_n = 0.793
        self.Mohr_cohesion = 1.3e3
        self.Mohr_friction = 31.1
        self.Janosi_shear = 1.2e-2
        self.elastic_K = 4e8
        self.damping_R = 3e4


chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')           # locate vehicle data files

init_loc = chrono.ChVector3d(-5, 0, 0.6)                            # spawn behind world origin, above SCM rest plane
init_rot = chrono.QuatFromAngleZ(0)                                # facing +X
step_size = 2e-3                                                    # integration step (s)

# HMMWV full model on deformable soil: SMC contact + non-rigid TMEASY tire.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                 # SMC for SCM/deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)             # chassis collision handled by SCM, not mesh
hmmwv.SetChassisFixed(False)                                      # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # SCM requires a non-rigid tire
hmmwv.SetTireStepSize(step_size)                                  # sub-step the tire force model
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                         # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before building SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())            # report total vehicle mass

terrainLength = 40.0                                               # SCM patch X size (m)
terrainWidth = 40.0                                                # SCM patch Y size (m)
delta = 0.05                                                       # SCM grid resolution (m)

# Build the deformable terrain and configure its soil from the encapsulated class.
terrain = veh.SCMTerrain(system)
terrain_params = SCMParameters()                                  # encapsulated parameter object
terrain_params.InitializeParametersAsMid()                        # predefined config: "mid"
terrain_params.SetParameters(terrain)                             # apply the 8 coefficients

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)        # color the sinkage heatmap
terrain.AddMovingPatch(                                            # update only cells near the chassis
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),
    chrono.ChVector3d(5, 3, 1),
)
terrain.Initialize(terrainLength, terrainWidth, delta)            # length, width, grid resolution
terrain.SetMeshWireframe(False)
terrain.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80)

# TMEASY tires carry no automatic collision geometry — add explicit spindle
# cylinders so SCM's ray-casts detect the wheels and form ruts.
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
system.GetCollisionSystem().BindAll()                             # rebuild all collision models after edits

# Vehicle-specific Irrlicht window (Initialize first, scene elements after, NO grid).
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on SCM deformable terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# Interactive driver bound to the visual system (the catalog-vehicle truth driver).
driver = veh.ChInteractiveDriverIRR(vis)
render_step_size = 1.0 / 50.0
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

sim_end = 10.0                                                     # total simulated time (s)
render_steps = math.ceil(render_step_size / step_size)            # physics steps between rendered frames
render_every = render_steps                                       # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()

    if step_number % render_steps == 0:                          # throttled rendering
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
    hmmwv.Advance(step_size)                                      # advances the wrapper-owned system
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                               # spin so wall-clock matches sim time
