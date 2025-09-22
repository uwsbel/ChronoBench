import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  

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