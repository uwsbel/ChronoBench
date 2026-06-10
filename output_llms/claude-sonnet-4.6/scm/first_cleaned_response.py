import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr




step_size = 1e-3          
tire_step_size = 1e-3     
render_fps = 50           
render_step_size = 1.0 / render_fps


terrainLength = 16.0      
terrainWidth = 8.0        


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)





veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


hmmwv = veh.HMMWV_Full()


hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)


hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))


hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)


hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(tire_step_size)


hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


vehicle_system = hmmwv.GetSystem()




terrain = veh.SCMTerrain(vehicle_system)



terrain.SetSoilParameters(
    2e6,   
    0,     
    1.1,   
    0,     
    30,    
    0.01,  
    2e8,   
    3e4    
)


terrain.EnableBulldozing(True)
terrain.SetBulldozingParameters(
    55,    
    1,     
    5,     
    3,     
    10     
)


terrain.AddMovingPatch(
    hmmwv.GetChassisBody(),          
    chrono.ChVector3d(0, 0, 0),      
    chrono.ChVector3d(5.0, 3.0, 1.0) 
)


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(
    terrainLength,    
    terrainWidth,     
    0.02              
)




driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())


driver.SetSteeringDelta(render_step_size / 1.0)   
driver.SetThrottleDelta(render_step_size / 8.0)   
driver.SetBrakingDelta(render_step_size / 2.0)    




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()


vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetWindowSize(1280, 720)


vis.SetChaseCamera(
    chrono.ChVector3d(0.0, 0.0, 0.75),  
    6.0,                                  
    0.5                                   
)


vis.Initialize()


vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_chrono_alpha.png')


vis.AttachVehicle(hmmwv.GetVehicleInstance())


driver.Initialize()




print("HMMWV SCM Terrain Simulation Starting...")
print(f"Step size: {step_size} s")
print(f"Render FPS: {render_fps}")
print("Use keyboard controls:")
print("  Arrow keys: Steering and throttle")
print("  Space: Braking")
print("  Q/ESC: Quit")


realtime_timer = chrono.ChRealtimeStepTimer()


sim_time = 0.0
render_time = 0.0
step_number = 0

while vis.Run():
    
    sim_time = hmmwv.GetSystem().GetChTime()

    
    if sim_time >= render_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_time += render_step_size

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    hmmwv.Synchronize(sim_time, driver_inputs, terrain)
    vis.Synchronize(sim_time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

print("Simulation completed.")