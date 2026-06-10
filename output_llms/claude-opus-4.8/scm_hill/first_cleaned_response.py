import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-5, -2, 0.6)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


step_size = 2e-3
tire_step_size = 1e-3






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
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


system = hmmwv.GetSystem()






terrain_length = 20.0   
terrain_width = 20.0    
delta = 0.05            


terrain = veh.SCMTerrain(system)


terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0),
                                    chrono.QuatFromAngleX(-math.pi / 2)))


terrain.Initialize(
    veh.GetDataFile("terrain/height_maps/test64.bmp"),  
    terrain_length,   
    terrain_width,    
    0.0,              
    1.0,              
    delta             
)


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
    6      
)


terrain.SetAutomaticRefinement(True)
terrain.SetAutomaticRefinementResolution(0.04)


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.GetMesh().SetWireframe(True)





driver = veh.ChInteractiveDriverIRR  





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


driver.SetSteeringDelta(0.04)   
driver.SetThrottleDelta(0.04)
driver.SetBrakingDelta(0.06)
driver.Initialize()






realtime_timer = chrono.ChRealtimeStepTimer()

print("Starting simulation...")

while vis.Run():
    time = system.GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

print("Simulation finished.")