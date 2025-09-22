import math
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')




initLoc = chrono.ChVector3d(-15, 0, 0.5)          
initRot = chrono.ChQuaterniond(1, 0, 0, 0)        




vis_type               = veh.VisualizationType_MESH
chassis_collision_type = veh.ChassisCollisionType_NONE          
tire_model             = veh.TireModelType_TMEASY




terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0




trackPoint = chrono.ChVector3d(3, 0, 2.1)         
chase_dist = 25.0                                 
chase_hgt  = 10.5                                 




contact_method     = chrono.ChContactMethod_NSC
step_size          = 1e-3
render_step_size   = 1.0 / 50.0                   
render_steps       = math.ceil(render_step_size / step_size)




vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type)        


vehicle.GetSystem().SetCollisionSystemType(
    chrono.ChCollisionSystem.Type_BULLET)




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())
patch   = terrain.AddPatch(
            patch_mat,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz – Double Lane Change')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, chase_dist, chase_hgt)   
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())                 




class DoubleLaneChangeDriver(veh.ChDriver):
    
    def __init__(self, vehicle):
        super(DoubleLaneChangeDriver, self).__init__(vehicle)
        self.throttle = 0.6

    
    def Synchronize(self, time):
        
        steer = 0.0
        
        if 1.0 <= time < 2.5:
            steer = 0.35 * (time - 1.0) / (2.5 - 1.0)           
        elif 2.5 <= time < 4.0:
            steer = 0.35 * (4.0 - time) / (4.0 - 2.5)           
        
        elif 4.0 <= time < 5.5:
            steer = -0.35 * (time - 4.0) / (5.5 - 4.0)          
        elif 5.5 <= time < 7.0:
            steer = -0.35 * (7.0 - time) / (7.0 - 5.5)          

        
        self.m_inputs.m_steering = steer
        self.m_inputs.m_throttle = self.throttle
        self.m_inputs.m_braking  = 0.0

    def Advance(self, step):
        pass   


driver = DoubleLaneChangeDriver(vehicle.GetTractor())
driver.Initialize()


print("VEHICLE MASS:", vehicle.GetTractor().GetMass())




realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0
render_frame   = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver.Synchronize(time)
    driver_inputs = driver.GetInputs()

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1
    realtime_timer.Spin(step_size)