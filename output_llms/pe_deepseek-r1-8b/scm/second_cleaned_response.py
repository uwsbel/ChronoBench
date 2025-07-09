import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


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



class SCMTerrainParams:
    def __init__(self):
        self.BekkerKphi = 2e6
        self.BekkerKc = 0
        self.Bekker_n = 1.1
        self.MohrCohesiveLimit = 0
        self.MohrFrictionLimit = 30
        self.JanosiShear = 0.01
        self.ElasticStiffness = 2e8
        self.Damping = 3e4

    def SetSoilParameters(self, BekkerKphi, BekkerKc, Bekker_n, MohrCohesiveLimit, MohrFrictionLimit, JanosiShear, ElasticStiffness, Damping):
        self.BekkerKphi = BekkerKphi
        self.BekkerKc = BekkerKc
        self.Bekker_n = Bekker_n
        self.MohrCohesiveLimit = MohrCohesiveLimit
        self.MohrFrictionLimit = MohrFrictionLimit
        self.JanosiShear = JanosiShear
        self.ElasticStiffness = ElasticStiffness
        self.Damping = Damping

    def InitializeSoft(self):
        self.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)

    def InitializeMid(self):
        self.SetSoilParameters(1e6, 0, 1.2, 0, 35, 0.02, 1e8, 5e4)

    def InitializeHard(self):
        self.SetSoilParameters(5e6, 0, 1.0, 0, 40, 0.005, 5e9, 1e5)


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)


terrain_params = SCMTerrainParams()
terrain_params.InitializeSoft()  
terrain = veh.SCMTerrain(vehicle.GetSystem(), terrain_params)

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(terrainLength, terrainWidth, 0.02)


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)


terrain_params = SCMTerrainParams()
terrain_params.InitializeSoft()  
terrain = veh.SCMTerrain(vehicle.GetSystem(), terrain_params)

terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.Initialize(terrainLength, terrainWidth, 0.02)