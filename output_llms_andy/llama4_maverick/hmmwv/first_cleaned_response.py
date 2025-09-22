import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print("Copyright (c) 2023 ProjectChrono")


system = chrono.ChSystemSMC()


init_loc = chrono.ChVector3d(0, 0, 1.0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
vehicle = veh.HMMWV_Full(system, veh.ChContactMethod_SMC, init_loc, init_rot)


tire_model = veh.TMeasyTire::Type.TMEASY_MF
vehicle.SetTireType(tire_model)


vehicle.SetInitWheelAngVel(chrono.ChVector3d(0, 0, 0))
vehicle.SetInitDriveAngVel(chrono.ChVector3d(0, 0, 0))
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetTireCollisionType(veh.ChTireCollisionType_ENVELOPE)


vehicle.Initialize()


vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_C_PI_2)), chrono.ChVector3d(100, 100, 1))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 100, 100)
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(10, 10, 5), chrono.ChVector3d(0, 0, 0))


driver = veh.ChIrrGuiDriver(vehicle, vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time, driver_inputs)
    terrain.Advance(time)
    system.DoStepDynamics(1e-3)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    realtime_timer.Spin(1.0 / 50.0)