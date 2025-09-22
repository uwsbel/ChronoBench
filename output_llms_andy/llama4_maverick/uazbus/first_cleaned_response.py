import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.Init()


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_C_PI_2)))
terrain.Initialize()


init_loc = chrono.ChVectorD(0, 0, 1.0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.UAZBUS(system, "UAZBUS/UAZBUS_UWA.json")
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChDriver(vehicle)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional(100, chrono.ChColor(1, 1, 1), chrono.ChVectorD(0, 0, 1))
vis.AddLightWithShadow(100, chrono.ChVectorD(2, 2.5, 2.5), chrono.ChVectorD(0, 0, 0), 10, 2, 10, 40, chrono.ChColor(0.8, 0.8, 0.8))
vis.AttachVehicle(vehicle)


realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(time, driver_inputs)
    system.DoStepDynamics(1e-3)
    vis.Synchronize("", driver_inputs)
    vis.Advance(1e-3)
    realtime_timer.Spin(1e-3)