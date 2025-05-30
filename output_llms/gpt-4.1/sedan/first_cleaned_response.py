import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())




step_size = 2e-3
render_step_size = 1.0 / 60  
tire_step_size = 1e-3




vehicle = veh.WheeledVehicle(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.2), chrono.QUNIT)
)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.2), chrono.QUNIT))




tire_FL = veh.TMeasyTire("FL")
tire_FR = veh.TMeasyTire("FR")
tire_RL = veh.TMeasyTire("RL")
tire_RR = veh.TMeasyTire("RR")
tire_FL.SetVisualizationType(veh.VisualizationType_MESH)
tire_FR.SetVisualizationType(veh.VisualizationType_MESH)
tire_RL.SetVisualizationType(veh.VisualizationType_MESH)
tire_RR.SetVisualizationType(veh.VisualizationType_MESH)

tire_FL.Initialize(vehicle.GetWheel(0, veh.VehicleSide_LEFT))
tire_FR.Initialize(vehicle.GetWheel(0, veh.VehicleSide_RIGHT))
tire_RL.Initialize(vehicle.GetWheel(1, veh.VehicleSide_LEFT))
tire_RR.Initialize(vehicle.GetWheel(1, veh.VehicleSide_RIGHT))




terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(200, 200, 1),  
    "concrete"
)

patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/tile4.jpg"), 200, 200)

patch.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/logo_chronoengine_alpha.png"), 10, 10)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()




driver = veh.ChIrrGuiDriver(
    veh.GetDataFile("UI/driver/driver.json")
)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()




vis = veh.ChWheeledVehicleIrrApp(
    vehicle, "BMW E90 Sedan on Rigid Terrain", irr.dimension2du(1280, 720)
)
vis.SetChaseCamera(
    chrono.ChVectorD(0.0, 0.0, 1.75),  
    6.0,  
    0.5   
)
vis.AddTypicalLights(chrono.ChVectorD(100, 100, 100), chrono.ChVectorD(-100, -100, 100), 250, 130)
vis.AddSkyBox()
vis.SetSymbolscale(1.0)
vis.SetLogLevel(chrono.GetLogLevel_INFO)
vis.AssetBindAll()
vis.AssetUpdateAll()




realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = int(render_step_size / step_size)
step_number = 0

while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(vehicle.GetSystem().GetChTime())
    terrain.Synchronize(vehicle.GetSystem().GetChTime())
    vehicle.Synchronize(vehicle.GetSystem().GetChTime(), driver_inputs, terrain)
    tire_FL.Synchronize(vehicle.GetSystem().GetChTime(), terrain)
    tire_FR.Synchronize(vehicle.GetSystem().GetChTime(), terrain)
    tire_RL.Synchronize(vehicle.GetSystem().GetChTime(), terrain)
    tire_RR.Synchronize(vehicle.GetSystem().GetChTime(), terrain)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    tire_FL.Advance(step_size)
    tire_FR.Advance(step_size)
    tire_RL.Advance(step_size)
    tire_RR.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1

vis.GetDevice().closeDevice()