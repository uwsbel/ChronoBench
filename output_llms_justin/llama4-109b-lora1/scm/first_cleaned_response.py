import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens


chrono.SetChairsThreading(True)
chrono.SetChronoDataFilePath(chrono.GetChronoDataFilePath() + '/vehicle/hmmwv/')


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
hmmwv.SetTireType(veh.TireModelType_Rigid)
hmmwv.SetTireStepSize(0.01)
hmmwv.EnableMeshVisualization(True, 1.0)


terrain = veh.GetTerrain(hmmwv.GetSystem())
terrain.SetType(veh.TerrainType_SCM)
terrain.SetCollideType(veh.CollideType_HeightMap)
terrain.SetSoilParameters(0.2, 0.3, 2.0, 0.01, 0.5, 2.0, 0.3, 0.5, 0.03, 0.5, 2.0)
terrain.EnableMovingPatch(True)
terrain.EnableColorPlot(True)
terrain.EnableAlphaBlend(True)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(hmmwv.GetSystem())
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.SetWindowSize(1280, 720)
vis.EnableShadow(True)
vis.EnableContactDrawing(vis.ContactDrawMode_COLLISION_POINTS)
vis.EnableRayTracing(True)


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.5 / 180)
driver.SetThrottleDelta(0.5 / 10)
driver.SetBrakingDelta(0.5 / 10)
driver.Initialize()


timestep = 0.02
realtime_factor = 1
hmmwv.Initialize()
terrain.Initialize()
vis.Initialize()


while vis.Run():
    time = hmmwv.GetSystem().GetChTime()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vis.Synchronize(time, driver.GetVehicle())
    vis.Render()
    hmmwv.GetSystem().DoStepDynamics(timestep)
    vis.UpdateCamera(chrono.ChVector3d(0, -5, 1), hmmwv.GetVehicle().GetChassisBody().GetPos())
    driver.Update()
    terrain.Update()
    vis.Update()
    vis.ShowStats()