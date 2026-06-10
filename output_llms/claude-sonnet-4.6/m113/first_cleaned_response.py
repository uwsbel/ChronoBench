import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






m113 = veh.M113()


init_loc = chrono.ChVector3d(0, 0, 1.1)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  

m113.SetContactMethod(chrono.ChContactMethod_SMC)
m113.SetChassisFixed(False)
m113.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
m113.SetBrakeType(veh.BrakeType_SIMPLE)
m113.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
m113.SetDrivelineType(veh.DrivelineTypeTV_BDS)
m113.SetEngineType(veh.EngineModelType_SIMPLE_MAP)
m113.SetTransmissionType(veh.TransmissionModelType_SIMPLE_MAP)


m113.Initialize()


m113.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSprocketVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetIdlerWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetRoadWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
m113.SetTrackShoeVisualizationType(veh.VisualizationType_PRIMITIVES)






terrain = veh.RigidTerrain(m113.GetSystem())


patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)          
patch_mat.SetRestitution(0.01)      
patch_mat.SetYoungModulus(2e7)      


patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    200.0,  
    200.0   
)


patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


terrain.Initialize()






driver = veh.ChInteractiveDriverIRR(vis := irr.ChVisualSystemIrrlicht())


driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(m113.GetSystem())
vis.SetWindowTitle('M113 Vehicle Simulation')
vis.SetWindowSize(1280, 720)
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()


vis.AddLightDirectional()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),  
    chrono.ChVector3d(0, 0, 0),           
    3,                                     
    2.2, 7.2,                             
    40,                                   
    512                                   
)


vis.SetChaseCamera(
    chrono.ChVector3d(0.0, 0.0, 0.0),  
    6.0,                                 
    0.5                                  
)
vis.SetChaseCameraState(irr.utils.ChChaseCamera.Chase)
vis.SetChaseCameraPosition(m113.GetChassisBody().GetPos() + chrono.ChVector3d(-6, 0, 2))






driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()






step_size = 5e-4


render_step_size = 1.0 / 50  
render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()





print("Starting M113 simulation...")
print("Use arrow keys to control the vehicle:")
print("  Up/Down    - Throttle/Brake")
print("  Left/Right - Steering")

step_number = 0
render_frame = 0

while vis.Run():
    
    time = m113.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    m113.Advance(step_size)
    vis.Advance(step_size)

    
    realtime_timer.Spin(step_size)

    
    step_number += 1

print("Simulation completed.")