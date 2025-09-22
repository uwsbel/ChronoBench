import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import time




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(veh.GetDataPath())


step_size = 2e-3
render_step_size = 1.0 / 50  
time_end = 10.0




contact_method = chrono.ChContactMethod_NSC
system = chrono.ChSystemNSC()


init_loc = chrono.ChVectorD(0, 0, 1.0)
init_yaw = 0 * chrono.CH_C_DEG_TO_RAD


vehicle = veh.HMMWV_Full(system,
                         contact_method,
                         chrono.vehicle.ChassisCollisionType_NONE)

vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, chrono.Q_from_AngZ(init_yaw)))
vehicle.SetChassisFixed(False)
vehicle.SetTireType(veh.TireType_TMEASY)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()


vehicle.SetVisualizationTypeChassis(veh.VisualizationType_PRIMITIVES)
vehicle.SetVisualizationTypeSuspension(veh.VisualizationType_PRIMITIVES)
vehicle.SetVisualizationTypeSteering(veh.VisualizationType_PRIMITIVES)
vehicle.SetVisualizationTypeWheel(veh.VisualizationType_PRIMITIVES)
vehicle.SetVisualizationTypeTire(veh.VisualizationType_PRIMITIVES)




terrain = veh.RigidTerrain(system)


patch = terrain.AddPatch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    length=200.0, width=100.0
)
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetMaterialSurface(contact_method)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 100)
terrain.Initialize()




vis = veh.ChWheeledVehicleIrrApp(vehicle.GetVehicle(), 'PyChrono HMMWV Demo', irr.dimension2du(1024,768))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddTypicalLogo()
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
vis.SetTimestep(step_size)




driver = veh.ChIrrGuiDriver(vis)

driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()




realtime_timer = chrono.ChRealtimeStepTimer()
render_steps = math.ceil(render_step_size / step_size)
step_number = 0

while vis.Run():
    if vehicle.GetVehicle().GetChTime() > time_end:
        break

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(vehicle.GetVehicle().GetChTime())
    terrain.Synchronize(vehicle.GetVehicle().GetChTime())
    vehicle.Synchronize(vehicle.GetVehicle().GetChTime(), driver_inputs, terrain)
    vis.Synchronize("HMMWV Demo", driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)
    step_number += 1