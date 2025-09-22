import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChairsThreading(True)
chrono.SetChronoDataPath("chrono_data/")


vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.1), chrono.Quatd(1, 0, 0, 0)))
vehicle.SetInitSpeed(0)
vehicle.SetInitOmega(0)
vehicle.SetTireVisualization(True)
vehicle.SetWheelVisualization(True)
vehicle.SetChassisVisualization(True)
vehicle.SetSuspensionVisualization(True)
vehicle.SetSteeringVisualization(True)
vehicle.SetDrivetrainVisualization(True)
vehicle.SetFrameRate(60)
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.ChCoordsysd(), 6.0, 6.0, 120, 120)
terrain.Initialize()


driver = veh.ChVehicleIrrGuiDriver(vehicle)
driver.SetSteeringDelta(0.5)
driver.SetThrottleDelta(0.5)
driver.SetBrakingDelta(0.5)


vis = irr.ChVisualSystemIrrLicht()
vis.AttachVehicle(vehicle)
vis.SetWindowTitle('Kraz Simulation')
vis.SetWindowSize(800, 600)
vis.SetChaseCamera(chrono.ChVector3d(0.0, -5.0, 1.5), 0.5, 0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 0), 0, 0)


step_size = 1e-3
t = 0
while vis.Run() and not driver.IsFinished():
    driver.Synchronize(time=t)
    terrain.Synchronize(time=t)
    vis.Synchronize(time=t)
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vis.Advance(step_size)
    t += step_size
vis.EndScene()