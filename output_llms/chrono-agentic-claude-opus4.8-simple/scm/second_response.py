import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

class SCMParameters:                                                   # encapsulates SCM soil parameters
    def __init__(self):
        self.Bekker_Kphi = 0.0                                         # frictional modulus (Pa)
        self.Bekker_Kc = 0.0                                           # cohesive modulus
        self.Bekker_n = 0.0                                            # sinkage exponent
        self.Mohr_cohesion = 0.0                                       # cohesive limit (Pa)
        self.Mohr_friction = 0.0                                       # friction angle (deg)
        self.Janosi_shear = 0.0                                        # shear modulus (m)
        self.elastic_K = 0.0                                           # elastic stiffness (Pa/m)
        self.damping_R = 0.0                                           # vertical damping (Pa·s/m)

    def SetParameters(self, terrain):                                  # push params onto the SCM terrain
        terrain.SetSoilParameters(self.Bekker_Kphi, self.Bekker_Kc, self.Bekker_n,
                                  self.Mohr_cohesion, self.Mohr_friction, self.Janosi_shear,
                                  self.elastic_K, self.damping_R)

    def InitializeParametersAsSoft(self):                              # soft soil preset
        self.Bekker_Kphi = 0.2e6
        self.Bekker_Kc = 0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0
        self.Mohr_friction = 30
        self.Janosi_shear = 0.01
        self.elastic_K = 4e7
        self.damping_R = 3e4

    def InitializeParametersAsMid(self):                               # medium soil preset
        self.Bekker_Kphi = 2e6
        self.Bekker_Kc = 0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0
        self.Mohr_friction = 30
        self.Janosi_shear = 0.01
        self.elastic_K = 2e8
        self.damping_R = 3e4

    def InitializeParametersAsHard(self):                              # hard soil preset
        self.Bekker_Kphi = 5301e3
        self.Bekker_Kc = 102e3
        self.Bekker_n = 0.793
        self.Mohr_cohesion = 1.3e3
        self.Mohr_friction = 31.1
        self.Janosi_shear = 1.2e-2
        self.elastic_K = 4e8
        self.damping_R = 3e4

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # core data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # vehicle data path

initLoc = chrono.ChVector3d(0, 0, 0.5)                                 # HMMWV spawn location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                            # QUNIT, no rotation

step_size = 1e-3                                                       # integration step
tire_step_size = step_size                                            # tire substep matches step

hmmwv = veh.HMMWV_Full()                                               # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)                     # SMC for SCM deformable terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                  # chassis has no collision shape
hmmwv.SetChassisFixed(False)                                           # chassis is free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))           # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire for SCM driving
hmmwv.SetTireStepSize(tire_step_size)                                 # tire integration step
hmmwv.Initialize()                                                     # build the vehicle

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # BULLET before SCM
system = hmmwv.GetSystem()                                             # the wrapper-owned ChSystem

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                 # truth vehicle banner

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)         # mesh visualization, all parts
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.SCMTerrain(system)                                       # deformable SCM terrain
terrain_params = SCMParameters()                                       # parameter object
terrain_params.InitializeParametersAsMid()                             # use the "mid" preset
terrain_params.SetParameters(terrain)                                  # apply preset to terrain
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.1)            # false-color sinkage plot
terrain.AddMovingPatch(                                                # moving patch follows chassis
    hmmwv.GetChassisBody(),
    chrono.ChVector3d(0, 0, 0),                                       # local OOBB centre
    chrono.ChVector3d(5, 3, 1),                                       # OOBB dimensions (m)
)
terrain.SetMeshWireframe(False)                                        # solid mesh, not wireframe
terrain.Initialize(20.0, 20.0, 0.02)                                  # length, width, grid resolution (m)

tire_rad = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetRadius()  # tire radius
tire_w = hmmwv.GetVehicle().GetAxles()[0].m_wheels[0].GetTire().GetWidth()     # tire width
tire_mat = chrono.ChContactMaterialSMC()                               # tire-soil contact material
tire_mat.SetFriction(0.9)
tire_mat.SetRestitution(0.1)
TIRE_FAMILY = 1                                                        # collision family for tires
for axle in hmmwv.GetVehicle().GetAxles():                            # explicit collision cylinders for TMEASY
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
system.GetCollisionSystem().BindAll()                                  # rebuild collision models

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # vehicle Irrlicht visual system
vis.SetWindowTitle('HMMWV Demo')                                       # window title
vis.SetWindowSize(1280, 1024)                                          # window pixels
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)           # chase camera trackpoint/dist/height
vis.Initialize()                                                       # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # logo (after Initialize)
vis.AddSkyBox()                                                        # sky box
vis.AddLightDirectional()                                              # directional light (vehicle default)
vis.AttachVehicle(hmmwv.GetVehicle())                                  # bind vehicle for chase camera

driver = veh.ChInteractiveDriverIRR(vis)                               # interactive keyboard driver
render_step_size = 1.0 / 50.0                                          # render at 50 fps
driver.SetSteeringDelta(render_step_size / 1.0)                        # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                        # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                         # braking rate
driver.Initialize()                                                    # build the driver

render_every = max(1, round(render_step_size / step_size))            # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                         # wall-clock pacing
step_number = 0                                                        # physics step counter
while vis.Run():                                                       # plain real-time loop
    time = system.GetChTime()                                         # current sim time

    if step_number % render_every == 0:                               # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # current driver inputs

    driver.Synchronize(time)                                         # synchronize subsystems
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)                                        # advance subsystems
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                                          # advances the wrapper system
    vis.Advance(step_size)

    step_number += 1                                                 # next step
    realtime_timer.Spin(step_size)                                   # spin to wall-clock
