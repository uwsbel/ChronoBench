import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


class SCMParameters:                                                 # encapsulate the 8 SCM soil params
    def __init__(self):
        self.Bekker_Kphi = 0.0                                       # frictional modulus (Pa)
        self.Bekker_Kc = 0.0                                         # cohesive modulus (Pa)
        self.Bekker_n = 0.0                                          # Bekker exponent
        self.Mohr_cohesion = 0.0                                     # Mohr cohesive limit (Pa)
        self.Mohr_friction = 0.0                                     # Mohr friction angle (deg)
        self.Janosi_shear = 0.0                                      # Janosi shear coefficient (m)
        self.elastic_K = 0.0                                         # elastic stiffness (Pa/m)
        self.damping_R = 0.0                                         # vertical damping (Pa*s/m)

    def SetParameters(self, terrain):                               # push the params onto an SCMTerrain
        terrain.SetSoilParameters(
            self.Bekker_Kphi,                                        # Bekker_Kphi
            self.Bekker_Kc,                                          # Bekker_Kc
            self.Bekker_n,                                           # Bekker_n
            self.Mohr_cohesion,                                      # Mohr_cohesion
            self.Mohr_friction,                                      # Mohr_friction
            self.Janosi_shear,                                       # Janosi_shear
            self.elastic_K,                                          # elastic_K
            self.damping_R,                                          # damping_R
        )

    def InitializeParametersAsSoft(self):                          # "soft" soil preset
        self.Bekker_Kphi = 0.2e6                                     # low frictional modulus -> deep sinkage
        self.Bekker_Kc = 0.0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0.0
        self.Mohr_friction = 30.0
        self.Janosi_shear = 0.01
        self.elastic_K = 4e7
        self.damping_R = 3e4

    def InitializeParametersAsMid(self):                           # "mid" soil preset
        self.Bekker_Kphi = 2e6                                      # moderate frictional modulus
        self.Bekker_Kc = 0.0
        self.Bekker_n = 1.0
        self.Mohr_cohesion = 0.0
        self.Mohr_friction = 30.0
        self.Janosi_shear = 0.01
        self.elastic_K = 2e8
        self.damping_R = 3e4

    def InitializeParametersAsHard(self):                          # "hard" soil preset
        self.Bekker_Kphi = 5301e3                                   # high frictional modulus -> firm ground
        self.Bekker_Kc = 102e3
        self.Bekker_n = 0.793
        self.Mohr_cohesion = 1.3e3
        self.Mohr_friction = 31.1
        self.Janosi_shear = 1.2e-2
        self.elastic_K = 4e8
        self.damping_R = 3e4


step_size = 2e-3                                                     # integration step (s)
tire_step_size = 1e-3                                                # TMEASY tire substep (s)
init_loc = chrono.ChVector3d(-5, 0, 0.6)                            # spawn near the patch edge
init_rot = chrono.QuatFromAngleZ(0)                                 # heading along +X

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')            # locate vehicle data files

hmmwv = veh.HMMWV_Full()                                            # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                  # SMC for deformable SCM soil
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)              # chassis collides via SCM only
hmmwv.SetChassisFixed(False)                                       # MANDATORY - fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # SCM requires a non-rigid tire
hmmwv.SetTireStepSize(tire_step_size)                              # tire force substep
hmmwv.Initialize()                                                 # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                         # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED before SCMTerrain
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.SCMTerrain(system)                                  # deformable Bekker-Wong terrain
params = SCMParameters()                                          # parameter manager object
params.InitializeParametersAsMid()                                # use the "mid" predefined config
params.SetParameters(terrain)                                     # apply the encapsulated soil params

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)        # colored sinkage heatmap
terrain.AddMovingPatch(                                           # only refine cells near the chassis
    hmmwv.GetChassisBody(),                                       # stable, level reference body
    chrono.ChVector3d(0, 0, 0),                                  # local OOBB center offset
    chrono.ChVector3d(5, 3, 1),                                  # OOBB dimensions (m)
)
terrain.Initialize(20.0, 20.0, 0.02)                             # length, width, grid resolution (m)
terrain.SetMeshWireframe(False)                                  # solid deformable mesh
terrain.SetTexture(                                              # dirt texture on the soil
    chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"), 80, 80,
)

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()   # tire radius (m)
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()      # tire width (m)
tire_mat = chrono.ChContactMaterialSMC()                         # SMC tire contact material
tire_mat.SetFriction(0.9)                                        # tire-soil friction
tire_mat.SetRestitution(0.1)                                     # tire restitution

TIRE_FAMILY = 1                                                  # collision family for tires
for axle in hmmwv.GetVehicle().GetAxles():                      # add a collision cylinder per wheel
    for iw in range(2):
        spindle = axle.m_wheels[iw].GetSpindle()                # the spindle body
        spindle.AddCollisionShape(                              # cylinder slightly below tread
            chrono.ChCollisionShapeCylinder(tire_mat, tire_rad + 0.04, tire_w),
            chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(math.pi / 2)),
        )
        spindle.EnableCollision(True)                          # let SCM ray-casts hit it
        sp_cm = spindle.GetCollisionModel()                    # the spindle collision model
        sp_cm.SetFamily(TIRE_FAMILY)                           # tag as tire family
        sp_cm.DisallowCollisionsWith(TIRE_FAMILY)             # wheels do not self-collide
system.GetCollisionSystem().BindAll()                          # rebuild models so cylinders are visible

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()               # vehicle-aware Irrlicht window
vis.SetWindowTitle("HMMWV on SCM deformable terrain")         # window title
vis.SetWindowSize(1280, 720)                                   # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)   # chase camera on the chassis
vis.Initialize()                                              # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                              # sky box backdrop
vis.AddLightDirectional()                                   # directional scene light
vis.AttachVehicle(hmmwv.GetVehicle())                       # bind chassis/wheel/tire assets

driver = veh.ChInteractiveDriverIRR(vis)                    # interactive driver bound to the window
render_step_size = 1.0 / 50.0                              # render at 50 FPS
driver.SetSteeringDelta(render_step_size / 1.0)           # 1 s to full steering
driver.SetThrottleDelta(render_step_size / 1.0)           # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)            # 0.3 s to full brake
driver.Initialize()                                        # finalize the driver

render_steps = math.ceil(render_step_size / step_size)    # physics steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()             # wall-clock pacing
step_number = 0                                            # physics step counter


while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                  # current sim time

    if step_number % render_steps == 0:                  # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                   # current driver inputs

    driver.Synchronize(time)                             # update driver
    terrain.Synchronize(time)                            # update soil
    hmmwv.Synchronize(time, driver_inputs, terrain)      # sample terrain at the wheels
    vis.Synchronize(time, driver_inputs)                 # update HUD/camera

    driver.Advance(step_size)                            # advance driver
    terrain.Advance(step_size)                           # advance soil deformation
    hmmwv.Advance(step_size)                             # advance wrapper-owned system
    vis.Advance(step_size)                               # advance visualization


    step_number += 1                                     # count this step
    realtime_timer.Spin(step_size)                       # pace to wall clock
