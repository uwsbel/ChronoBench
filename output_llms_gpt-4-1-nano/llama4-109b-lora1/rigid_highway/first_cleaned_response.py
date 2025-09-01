import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.vehicle.utils as veh_utils


chrono.SetChairsThreading(True)
chrono.SetChronoDataFilePath(chrono.GetChronoDataFilePath() + 'vehicle/hmmwv/')


vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on a Mesh Terrain')
vis.SetWindowSize(1280, 1024)
vis.SetRenderMode(irr.RenderMode_VITR)
vis.Initialize()


vehicle = veh.ChHMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(veh.ChCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetInitOrientation(chrono.ChQuaterniond(1, 0, 0, 0))
vehicle.SetTireType(veh.HMMWV_TireType_TMEASY)
vehicle.SetTireStepSize(0.02)
vehicle.EnableRealtime(True)
vehicle.Initialize()


terrain = veh.ChTerrain()
terrain.SetCollisionSystemType(veh.ChCollisionSystemType_BULLET)
terrain.SetVisualization(veh.TerrainType_COLLISION)
terrain.InitializeFromFile('data/vehicle/hmmwv/meshes/Highway_col.obj', 1.0)


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75))


vis.AddLogo(chrono.GetChronoDataFilePath() + 'logo_pychrono_alpha.png')
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, -10.0, 0.5), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.EnableDialogs()


step_size = 0.02
real_time_simulation = True


while vis.Run() and not vis.WasUserQuit():
    time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    terrain.Synchronize(time)
    vis.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    vis.Render()
    if real_time_simulation:
        vis.Sleep(step_size)