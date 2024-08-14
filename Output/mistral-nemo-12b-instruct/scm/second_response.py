import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

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
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)

# Contact method
contact_method = chrono.ChContactMethod_SMC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the HMMWV vehicle, set parameters, and initialize
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

# Create SCM terrain parameters class
class SCMTerrainParams:
    def __init__(self, kphi, kc, n, cohesive_limit, friction_limit, shear_coeff, elastic_stiffness, damping):
        self.kphi = kphi
        self.kc = kc
        self.n = n
        self.cohesive_limit = cohesive_limit
        self.friction_limit = friction_limit
        self.shear_coeff = shear_coeff
        self.elastic_stiffness = elastic_stiffness
        self.damping = damping

    def set_terrain(self, terrain):
        terrain.SetSoilParameters(self.kphi, self.kc, self.n, self.cohesive_limit, self.friction_limit, self.shear_coeff, self.elastic_stiffness, self.damping)

# Predefined terrain configurations
terrain_configs = {
    "soft": SCMTerrainParams(1e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4),
    "mid": SCMTerrainParams(1.5e6, 0, 1.2, 0, 35, 0.015, 3e8, 4e4),
    "hard": SCMTerrainParams(2e6, 0, 1.3, 0, 40, 0.02, 4e8, 5e4)
}

# Create the SCM deformable terrain patch with predefined configuration
terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain_configs["mid"].set_terrain(terrain)

# Optionally, enable moving patch feature (single patch around vehicle chassis)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))

# Set plot type for SCM (false color plotting)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

# Initialize the SCM terrain (length, width, mesh resolution), specifying the initial mesh grid
terrain.Initialize(20, 20, 0.02)

# ... (rest of the script remains the same)