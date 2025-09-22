import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np






class MyPathFollowerArcCurvature (veh.PathFollowerArcCurvature):
    def __init__(self, center, radius, velocity):
        veh.PathFollowerArcCurvature.__init__(self, center, radius, velocity)

    def GetPathName(self):
        return "Test path"

veh.SetDataPath(chrono.GetChronoDataFile(''))





initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_PRIMITIVES


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY



terrainHeight = 0      
terrainLength = 200.0  
terrainWidth = 200.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  




my_path = MyPathFollowerArcCurvature(chrono.ChVector3d(0, 0, 0), 8, 3)


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


driver = veh.ChPathFollowerDriverIRR(vis, my_path, 1000.0)


steering_time = 0.5  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringTime(steering_time)
driver.SetThrottleTime(throttle_time)
driver.SetBrakingTime(braking_time)
driver.Initialize()


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


try:
    os.mkdir("output")
except:
    print("output directory already exists\n")


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0





ball1 = vis.GetStratos()
ball1.SetName("Target ball")
ball1.SetChassisVisualizationType(veh.VisualizationType_NONE)
ball1.SetSuspensionVisualizationType(veh.VisualizationType_NONE)
ball1.SetSteeringVisualizationType(veh.VisualizationType_NONE)
ball1.SetWheelVisualizationType(veh.VisualizationType_NONE)
ball1.SetTireVisualizationType(veh.VisualizationType_NONE)
ball1.GetVehicle().SetChassisFixed(True)
ball1.Initialize()
vis.BindAll()

ball2 = vis.GetStratos()
ball2.SetName("Sentinel ball")
ball2.SetChassisVisualizationType(veh.VisualizationType_NONE)
ball2.SetSuspensionVisualizationType(veh.VisualizationType_NONE)
ball2.SetSteeringVisualizationType(veh.VisualizationType_NONE)
ball2.SetWheelVisualizationType(veh.VisualizationType_NONE)
ball2.SetTireVisualizationType(veh.VisualizationType_NONE)
ball2.GetVehicle().SetChassisFixed(True)
ball2.Initialize()
vis.BindAll()

ball1.GetSystem().SetCollide(False)
ball2.GetSystem().SetCollide(False)

ball1.GetVehicle().GetChassis().SetPos(chrono.ChVector3d(0, -8, 1))
ball2.GetVehicle().GetChassis().SetPos(chrono.ChVector3d(0, -8, 1))

ball1.GetVehicle().GetChassis().SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
ball2.GetVehicle().GetChassis().SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

ball1.GetChassisBody().AddVisualShape(chrono.ChVisualShapeSphere(0.1, chrono.ChColor(1,0,0), True))
ball2.GetChassisBody().AddVisualShape(chrono.ChVisualShapeSphere(0.1, chrono.ChColor(0,1,0), True))

ball1.GetSystem().EnableRealtime(True)
ball2.GetSystem().EnableRealtime(True)

ball1.GetVehicle().EnableRealtime(True)
ball2.GetVehicle().EnableRealtime(True)



while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    ball1.Synchronize(time, driver_inputs, terrain)
    ball2.Synchronize(time, driver_inputs, terrain)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    ball1.Advance(step_size)
    ball2.Advance(step_size)

    
    sphere1loc = ball1.GetChassisBody().GetPos()
    sphere2loc = ball2.GetChassisBody().GetPos()
    driver.Update(sphere1loc, sphere2loc)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

    
    if (driver.GetDriverStatus() == veh.DRIVER_STATUS_END):
        break