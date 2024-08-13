import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


class SCMParameters:
    def __init__(self):
        self.Bekker_Kphi = 0    # Kphi, frictional modulus in Bekker model
        self.Bekker_Kc = 0      # Kc, cohesive modulus in Bekker model
        self.Bekker_n = 0       # n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        self.Mohr_cohesion = 0  # Cohesion in, Pa, for shear failure
        self.Mohr_friction = 0  # Friction angle (in degrees!), for shear failure
        self.Janosi_shear = 0   # J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
        self.elastic_K = 0      # elastic stiffness K (must be > Kphi very high values gives the original SCM model)
        self.damping_R = 0      # vertical damping R, per unit area (vertical speed proportional, it is zero in original SCM model)

    def SetParameters(self, terrain):
        terrain.SetSoilParameters(
            self.Bekker_Kphi,    # Bekker Kphi
            self.Bekker_Kc,      # Bekker Kc
            self.Bekker_n,       # Bekker n exponent
            self.Mohr_cohesion,  # Mohr cohesive limit (Pa)
            self.Mohr_friction,  # Mohr friction limit (degrees)
            self.Janosi_shear,   # Janosi shear coefficient (m)
            self.elastic_K,      # Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
            self.damping_R)      # Damping (Pa s/m), proportional to negative vertical speed (optional)


class TerrainParameters:
    def __init__(self, terrain_params):
        self.terrain_params = terrain_params

    def InitializeParametersAsSoft(self):
        self.terrain_params.Bekker_Kphi = 0.2e6
        self.terrain_params.Bekker_Kc = 0
        self.terrain_params.Bekker_n = 1.1
        self.terrain_params.Mohr_cohesion = 0
        self.terrain_params.Mohr_friction = 30
        self.terrain_params.Janosi_shear = 0.01
        self.terrain_params.elastic_K = 4e7
        self.terrain_params.damping_R = 3e4

    def InitializeParametersAsMid(self):
        self.terrain_params.Bekker_Kphi = 2e6
        self.terrain_params.Bekker_Kc = 0
        self.terrain_params.Bekker_n = 1.1
        self.terrain_params.Mohr_cohesion = 0
        self.terrain_params.Mohr_friction = 30
        self.terrain_params.Janosi_shear = 0.01
        self.terrain_params.elastic_K = 2e8
        self.terrain_params.damping_R = 3e4

    def InitializeParametersAsHard(self):
        self.terrain_params.Bekker_Kphi = 5301e3
        self.terrain_params.Bekker_Kc = 102e3
        self.terrain_params.Bekker_n = 0.793
        self.terrain_params.Mohr_cohesion = 1.3e3
        self.terrain_params.Mohr_friction = 31.1
        self.terrain_params.Janosi_shear = 1.2e-2
        self.terrain_params.elastic_K = 4e8
        self.terrain_params.damping_R = 3e4


# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-8, 0, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_RIGID

# Rigid terrain
terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction






ic4















a
ic


















4


  



6_
print("error happened with only start ```python")