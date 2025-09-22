import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_PRIMITIVES

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

class MyDriver(veh.ChDriver):
    def __init__(self, delay=0.5):
        super().__init__()
        self.delay = delay
        self.throttle_target = 0
        self.steering_target = 0
        self.steering_time = 2.0
        self.steering_counter = 0

    def Synchronize(self, time):
        if time < self.delay:
            self.throttle_target = 0
            self.steering_target = 0
        elif time < self.delay + 0.2:
            self.throttle_target = 0.7 * (time - self.delay) / 0.2
            self.steering_target = 0
        else:
            self.throttle_target = 0.7
            self.steering_counter += 1
            if self.steering_counter % 10 == 0:
                self.steering_target = math.sin(time - self.steering_time)