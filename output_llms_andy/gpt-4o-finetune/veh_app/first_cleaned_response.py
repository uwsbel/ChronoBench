import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math



chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Vehicle Demo: Gator')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 34, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.SetLightIntensity(1.0)


light_pos = chrono.ChVector3f(2.0, 4.0, 2.0)
vis.GetSceneManager().addLightSource(None, light_pos, chrono.ChColor(1, 1, 1), 0.8)


init_pos = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.6), chrono.QuatFromAngleZ(0))
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(init_pos)
vehicle.SetTireType(veh.TireType.RIGID)
vehicle.SetTireStepSize(timestep)
vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


point_light = sens.ChLightManager().AddPointLight(intensity=1.0, color=chrono.ChColor(1, 1, 1),
                                                   abs_coords=light_pos, specular_coefficient=2)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(sys)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         100.0, 100.0)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

driver = veh.ChInteractiveDriverIRR(vis)


driver.SetSpeed(6.0)


manager = sens.ChSensorManager(sys)

light_list = vis.GetSceneManager().getLights()
for light in light_list:
    mgr_lgt = manager.AddLight(RemoveFromScene=False)
    mgr_lgt.SetIntensity(light.Intensity)
    mgr_lgt.SetLightColor(chrono.ChColor(light.Color.r, light.Color.g, light.Color.b))
    mgr_lgt.SetCoords(chrono.ChVector3f(light.Position.x, light.Position.y, light.Position.z))
    mgr_lgt.EnableCastShadows(light.CastShadows)
mgr_lgt = manager.AddLight(sens.ChLightData())
mgr_lgt.SetType(chrono.guid.SpatialApiPointLight)


cam = sens.ChCameraSensor(
    vehicle.GetChassisBody(),
    update_rate,
    sens.chrono.ChFrustum(chrono.ChRectangle(-1, 1, -0.75, 0.75), 1),  
    vis_width,
    vis_height,
    vis_filter)
cam.SetName("Gator POV Camera")
cam.SetLag(0)
cam.SetOffsetPose(chrono.ChFramed(chrono.ChVector3d(2, 0, 1.4), chrono.QuatFromAngleAxis(chrono.CH_PI_4, chrono.VECT_Y)))


if vis:
    cam.PushFilter(sens.ChFilterVisualize(vis_width, vis_height, "Gator POV"))
cam.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(cam)





frame = 0
time = 0


ts = vehicle.GetSystem().GetStepSize()


realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = vehicle.GetChassisBody().GetPos().Length()

    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    manager.Update()
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    driver.Advance(ts)
    terrain.Advance(ts)
    vehicle.Advance(ts)
    
    frame += 1

    
    sys.DoStepDynamics(ts)

    
    realtime_timer.Spin(ts)