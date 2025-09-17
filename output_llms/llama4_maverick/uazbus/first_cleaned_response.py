import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.Init()


system = chrono.ChSystemNSC()


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_C_PI_2)), chrono.ChVectorD(20, 20, 1), 0.01)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
terrain.Initialize()


initLoc = chrono.ChVectorD(0, 0, 1)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.UAZBUS(system, "UAZBUS/UAZBUS_UW.json")
vehicle.Initialize(chrono.ChFrameD(initLoc, initRot))
vehicle.SetChassisFixed(False)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('UAZBUS Demo')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional(chrono.ChVectorD(0, 0, -1), chrono.ChVectorD(0.5, 0.5, 0.5), 10)
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 1.5, 1.5), chrono.ChVectorD(0, 0, 0), 10, 1, 10, 40, chrono.ChColor(0.8, 0.8, 0.8))
vis.AttachVehicle(vehicle.GetVehicle())


while vis.Run():
    time = system.GetChTime()
    driver_inputs = driver.GetInputs()
    vehicle.Advance(0.01)
    system.DoStepDynamics(0.01)
    terrain.Advance(0.01)
    vis.Synchronize("", driver_inputs)
    vis.Advance(0.01)