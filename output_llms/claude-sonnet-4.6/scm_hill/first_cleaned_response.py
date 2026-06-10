import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math






chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')






initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(tire_model)
hmmwv.SetTireStepSize(1e-3)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)






step_size = 3e-3


terrainHeight = 0
terrainLength = 16.0  
terrainWidth = 8.0    



Kphi    = 2e6    
Kc      = 0      
n       = 1.1    
c       = 0      
phi     = 30     
K       = 0.01   
E_elastic = 2e8  
damping   = 3e4  


terrain = veh.SCMTerrain(hmmwv.GetSystem())
terrain.SetSoilParameters(Kphi, Kc, n, c, phi, K, E_elastic, damping)


terrain.EnableBulldozing(True)
terrain.SetBulldozingParameters(
    55,   
    1,    
    5,    
    10    
)


terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 30000.2)



terrain.Initialize(terrainLength, terrainWidth, 0.02)


terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)






driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())


steering_time = 1.0  
throttle_time = 1.0  
braking_time  = 0.3  

driver.SetSteeringDelta(step_size / steering_time)
driver.SetThrottleDelta(step_size / throttle_time)
driver.SetBrakingDelta(step_size / braking_time)





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.SetWindowSize(1280, 720)


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.75)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.SetChaseCameraState(veh.utils.ChChaseCamera.Chase)
vis.SetChaseCameraAngle(-math.pi / 6)


vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


vis.AttachVehicle(hmmwv.GetVehicle())
driver.Initialize()






print("Vehicle mass: ", hmmwv.GetVehicle().GetMass())


render_steps = math.ceil(1.0 / 50 / step_size)  


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0
time = 0

print("Starting simulation...")

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)

print("Simulation completed.")