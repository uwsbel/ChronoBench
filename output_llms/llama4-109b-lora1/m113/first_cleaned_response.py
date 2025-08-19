import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


ch = chrono.ChEngine('SCM')
ch.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
ch.SetSolverType(chrono.ChSolver.Type_PYMKL)
ch.SetSolverMaxIterations(100)


terrain = veh.RigidTerrain(ch.GetSystem())
terrain.SetCollisionMaterial(chrono.ChContactMaterialNSC())
terrain.SetContactFriction(0.9)
terrain.SetContactRestitution(0.3)
terrain.SetContactInertiaReduction(chrono.ChContactInertiaReduction.IR_ALWAYS_REDUCE)
terrain.SetContactCfm(0.02)
terrain.SetContactTimestep(1e-3)
terrain.Initialize()


vehicle = veh.M113(ch.GetSystem())
vehicle.Initialize()
vehicle.SetChassisVisualizationType(veh.VisualizationType_VITESSA)
vehicle.SetChassisRearVisualizationType(veh.VisualizationType_VITESSA)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_VITESSA)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_VITESSA)
vehicle.SetWheelVisualizationType(veh.VisualizationType_VITESSA)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
vehicle.SetDriverType(veh.DriverType_PAYNE)
vehicle.SetTireType(veh.TireType_TME)
vehicle.SetTireStepSize(1e-3)
vehicle.SetTireCollisionType(veh.TireCollisionType_RAY)
vehicle.SetTireCollisionFriction(0.8)
vehicle.SetTireCollisionRestitution(0.1)
vehicle.SetTireCollisionMaterial(chrono.ChContactMaterialNSC())
vehicle.SetTireCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle.SetTireCollisionSystemFriction(0.8)
vehicle.SetTireCollisionSystemRestitution(0.1)
vehicle.SetTireCollisionSystemMaterial(chrono.ChContactMaterialNSC())


vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetInitSpeed(0)
vehicle.SetInitOmega(chrono.ChVector3d(0, 0, 0))


driver = veh.PayneM113Driver(vehicle.GetVehicle())
driver.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachVehicle(vehicle.GetVehicle())
vis.SetWindowTitle('M113 Vehicle')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, -5.0, 1.5), 0.5, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
vis.AddLightDirectional(chrono.ChVector3d(0.5, 0.5, 0.5), chrono.ChVector3d(-0.5, -1, -0.5), True, 0.8)
vis.AddLightPoint(chrono.ChVector3d(-2, 0, 2), chrono.ChVector3d(0.0, 0.0, 0.0), 0.5)


step_size = 1e-3
t = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    vehicle.GetSystem().DoStepDynamics(step_size)
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vis.Synchronize(t, driver.GetPosRender())
    t += step_size
    if vis.GetKeyboard().KeyIsPressed(irr.KEY_ESCAPE):
        break