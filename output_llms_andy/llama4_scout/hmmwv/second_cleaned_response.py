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
terrainLength = 200.0  
terrainWidth = 100.0  


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


class PathFollower:
    def __init__(self, vehicle, vis):
        self.vehicle = vehicle
        self.vis = vis
        self.target_speed = 10.0  
        self.throttle_value = 0.3
        self.steering_pid = chrono.ChPidController(0.5, 0.1, 0.01)

    def update(self, time):
        
        pos = self.vehicle.GetVehicle().GetPos()
        speed = self.vehicle.GetVehicle().GetSpeed()

        
        radius = 20.0
        target_x = radius * math.cos(0.1 * time)
        target_y = radius * math.sin(0.1 * time)
        target_pos = chrono.ChVector3d(target_x, target_y, pos.z())

        
        steering_input = self.steering_pid.Update(pos.x() - target_x, pos.y() - target_y, render_step_size)

        
        steering_input = max(-1.0, min(steering_input, 1.0))

        
        driver_inputs = veh.DriverInputs()
        driver_inputs.SetThrottle(self.throttle_value)
        driver_inputs.SetSteering(steering_input)
        driver_inputs.SetBraking(0.0)

        return driver_inputs


path_follower = PathFollower(vehicle, vis)


sentinel_pos = chrono.ChVector3d(-10.0, 0.0, 1.0)
target_pos = chrono.ChVector3d(0.0, 0.0, 1.0)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % math.ceil(render_step_size / step_size) == 0:
        vis.BeginScene()

        
        vis.DrawSphere(sentinel_pos, 0.1, chrono.ChColor(1.0, 0.0, 0.0))
        vis.DrawSphere(target_pos, 0.1, chrono.ChColor(0.0, 1.0, 0.0))

        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = path_follower.update(time)

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

    
    print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())