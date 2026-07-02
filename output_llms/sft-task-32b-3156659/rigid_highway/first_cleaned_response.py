import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


hmmwv = vehicle.HMMWV_Full()
hmmwv.SetContactMethod(vehicle.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(vehicle.ChTireModelType_TMEASY)
hmmwv.SetTireStepSize(1e-3)
hmmwv.SetVisualizationType(vehicle.VisualizationType_MESH)


hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(vehicle.VisualizationType_MESH)


terrain_body = chrono.ChBodyEasyMesh("Highway_col.obj", 1000, True, True)
terrain_body.SetName("Highway Terrain")
terrain_body.SetFixed(True)
system.Add(terrain_body)

visual_shape = chrono.ChVisualShapeMesh()
visual_shape.SetFilename("Highway_vis.obj")
visual_shape.SetMutable(False)
terrain_body.AddVisualShape(visual_shape)


application = irr.ChIrrApp(system, "HMMWV Simulation", irr.dimension2du(1280, 720))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalCamera(irr.vector3df(0, 1.5, -6))
application.AddTypicalLights()
application.SetShowInfos(True)


driver = vehicle.ChIrrGuiDriver(application)
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)


time_step = 1.0 / 50.0
time = 0.0


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    driver_inputs = driver.GetInputs()
    hmmwv.SetDriverInputs(driver_inputs)
    
    
    system.DoStepDynamics(time_step)
    time += time_step
    
    application.EndScene()

    
    chrono.ChRealtimeStepDamping(time_step)