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
patch = terrain.AddPatch(patch_mat,chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),terrainLength, terrainWidth)
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



path = veh.ChLinePath()
radius = 20
for i in range(100) :
    theta = 2 * math.pi * i / 100.0
    path.AddPoint(veh.ChPathPoint(chrono.ChVector3d(radius * math.cos(theta), radius * math.sin(theta), 0), chrono.QUNIT, 0))

path.SetClosed(True)


point1 = chrono.ChVector3d()
point2 = chrono.ChVector3d()
for i in range(path.GetNumPoints() - 1) :
    path.EvalPoint(i + 0.5, point1)
    path.EvalPointDerivative(i + 0.5, point2)
    dir_vec = abs(point2 - point1)
    mball = chrono.ChVisualShapeBall(0.2)
    mball.SetMutable(False)
    mball.SetPos(point1 + dir_vec * 0.05)
    vis.GetScene().AddVisualShape(mball)
    mball = chrono.ChVisualShapeBall(0.2)
    mball.SetMutable(False)
    mball.SetPos(point1 + dir_vec * 0.95)
    vis.GetScene().AddVisualShape(mball)


controller = veh.ChPathFollowerPID()
end_speed = 0
wnd1 = 10
wnd2 = 10
controller.SetPath(path)
controller.SetLookAheadDistance(2)
controller.SetGains(wnd1, wnd2, wnd1, wnd2, wnd2)
controller.Initialize()


constant_throttle = 0.3


sentinel = chrono.ChVector3d()
target = chrono.ChVector3d()

msent = chrono.ChVisualShapeSphere(0.1)
msent.SetMutable(False)
vis.GetScene().AddVisualShape(msent)
mtarg = chrono.ChVisualShapeSphere(0.1)
mtarg.SetMutable(False)
vis.GetScene().AddVisualShape(mtarg)


print( "VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    
    driver_inputs = veh.ChDriverInputs()
    driver_inputs.m_throttle = constant_throttle

    
    veh_state = veh.ChVehicleRootState()

    
    controller.Synchronize(time, veh_state, driver_inputs)
    driver_inputs.m_steering = controller.GetSteering()

    
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)