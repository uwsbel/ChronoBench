import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_PRIMITIVES


tire_model = veh.TireModelType_TMEASY


terrainLength = 100.0  
terrainWidth = 100.0   

trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

contact_method = chrono.ChContactMethod_NSC
contact_vis = False

step_size = 1e-3
tire_step_size = step_size

render_step_size = 1.0 / 50  


vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
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


patch1 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(-25, -25, 0), chrono.QUNIT),
    50, 50)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile1.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


def slope_height(x, y):
    return 0.05 * x  
patch2 = terrain.AddHeightPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(25, -25, 0), chrono.QUNIT),
    50, 50, slope_height)
patch2.SetTexture(veh.GetDataFile("terrain/textures/tile2.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.6, 0.6, 0.3))


def bump_height(x, y):
    return 2.0 * math.exp(-(x**2 + y**2)/(2*10**2))
patch3 = terrain.AddHeightPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(-25, 25, 0), chrono.QUNIT),
    50, 50, bump_height)
patch3.SetTexture(veh.GetDataFile("terrain/textures/tile3.jpg"), 200, 200)
patch3.SetColor(chrono.ChColor(0.7, 0.5, 0.3))


patch4 = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(25, 25, 0), chrono.QUNIT),
    50, 50)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch4.SetColor(chrono.ChColor(0.8, 0.6, 0.4))

terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator vehicle')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

print("VEHICLE MASS: ", vehicle.GetMass())

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)