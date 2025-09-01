import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

class SCMParameters:
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
        terrain.SetSoilParameters(
            self.Bekker_Kphi,
            self.Bekker_Kc,
            self.Bekker_n,
            self.Mohr_cohesion,
            self.Mohr_friction,
            self.Janosi_shear,
            self.elastic_K,
            self.damping_R)

    def InitializeParametersAsSoft(self):
        self.Bekker_Kphi = 0.2e6
        self.Bekker_Kc = 0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0
        self.Mohr_friction = 30
        self.Janosi_shear = 0.01
        self.elastic_K = 4e7
        self.damping_R = 3e4

    def InitializeParametersAsMid(self):
        self.Bekker_Kphi = 2e6
        self.Bekker_Kc = 0
        self.Bekker_n = 1.1
        self.Mohr_cohesion = 0
        self.Mohr_friction = 30
        self.Janosi_shear = 0.01
        self.elastic_K = 2e8
        self.damping_R = 3e4

    def InitializeParametersAsHard(self):
        self.Bekker_Kphi = 5301e3
        self.Bekker_Kc = 102e3
        self.Bekker_n = 0.793
        self.Mohr_cohesion = 1.3e3
        self.Mohr_friction = 31.1
        self.Janosi_shear = 1.2e-2
        self.elastic_K = 4e8
        self.damping_R = 3e4

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH

terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0

contact_method = chrono.ChContactMethod_SMC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size

render_step_size = 1.0 / 50

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(collision_type)
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

terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain_params = SCMParameters()
terrain_params.InitializeParametersAsMid()
terrain_params.SetParameters(terrain)

terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(20, 20, 0.02)

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(track
print("error happened with only start ```python")