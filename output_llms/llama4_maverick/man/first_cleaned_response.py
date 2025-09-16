import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


print("Copyright (c) 2023 ProjectChrono")


out_dir = chrono.GetChronoOutputPath() + "MAN_10T_TRUCK_DEMO"
chrono.SetChronoOutputPath(out_dir)


veh_sys = veh.ChVehicleSystem(veh.ChVehicleSystem.Severity::WARNING)


truck = veh.MAN_10t(veh_sys)


truck.SetContactMethod(chrono.ChContactMethod::SMC)
truck.SetChassisCollisionType(veh.CollisionType::NONE)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-50, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
truck.SetTireType(veh.TireType::TMEASY)
truck.SetTireStepSize(1e-3)
truck.Initialize()


terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChVector3d(200, 200, 0))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetContactMaterialProperties(2e7, 0.3)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 400, 400)
terrain.Initialize()


road = veh.RigidTerrain(truck.GetSystem())
road.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChVector3d(200, 200, 0))


vis = irr.ChVisualSystemIrrlicht()
vis.AttachVehicleSystem(truck.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("MAN 10t Truck Demo")
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddDirectionalLight(chrono.ChVector3d(-1, -1, -1), 1.1)
vis.AddTypicalLights()


while vis.Run():
    time = truck.GetSystem().GetChTime()
    driver_inputs = veh.DriverInputs()
    
    driver_inputs.m_steering = 0.0
    driver_inputs.m_throttle = 0.5
    driver_inputs.m_braking = 0.0

    
    truck.GetSystem().Update(time, driver_inputs)
    terrain.Synchronize(time)
    vis.Synchronize("MAN 10t Truck Demo", driver_inputs)

    
    truck.GetSystem().Advance(1e-3)
    vis.Advance(1e-3)
    vis.Render()