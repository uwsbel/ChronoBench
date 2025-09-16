import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class SCMTerrainParams:
    def __init__(self, kphi, kc, n, cohesive_limit, friction_limit, shear_coefficient, elastic_stiffness, damping):
        self.kphi = kphi
        self.kc = kc
        self.n = n
        self.cohesive_limit = cohesive_limit
        self.friction_limit = friction_limit
        self.shear_coefficient = shear_coefficient
        self.elastic_stiffness = elastic_stiffness
        self.damping = damping


def initialize_scm_terrain(terrain, params):
    terrain.SetSoilParameters(
        params.kphi,
        params.kc,
        params.n,
        params.cohesive_limit,
        params.friction_limit,
        params.shear_coefficient,
        params.elastic_stiffness,
        params.damping
    )


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID

terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

contact_method = chrono.ChContactMethod_SMC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size
render_step_size = 1.0 / 50

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

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

terrain_params = SCMTerrainParams(
    2e6,  # Bekker Kphi
    0,    # Bekker Kc
    1.1,  # Bekker n exponent
    0,    # Mohr cohesive limit (Pa)
    30,   # Mohr friction limit (degrees)
    0.01, # Janosi shear coefficient (m)
    2e8,  # Elastic stiffness (Pa/m), before plastic yield
    3e4   # Damping (Pa s/m), proportional to negative vertical speed (optional)
)

terrain = veh.SCMTerrain(vehicle.GetSystem())
initialize_scm_terrain(terrain, terrain_params)

terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)