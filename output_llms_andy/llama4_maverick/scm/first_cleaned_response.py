import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math






output_dir = 'HMMWV_SCM_OUTPUT'


step_size = 2e-3


render_step_size = 1.0 / 50






veh.SetChronoDataPath('/path/to/chrono/data/')


init_loc = chrono.ChVectorD(0, 0, 1.0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.HMMWV_Full(
    init_loc,
    init_rot,
    True,
    veh.HMMWV_TireType_Rigid,
    veh.HMMWV_TireType_Rigid,
    veh.HMMWV_VisType_MESH,
    veh.HMMWV_VisType_MESH)

vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisFixed(False)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetSubchassisFixed(False)
vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
vehicle.SetTireCollisionType(veh.CollisionType_PRIMITIVES)
vehicle.SetInitWheelAngVel(chrono.ChVectorD(0, 0, -50))

vehicle.Initialize()

vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)






terrain = veh.SCMDeformableTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(
    2e6,    
    0,      
    1.1,    
    0,      
    30,     
    0,      
    4e4,    
    3e4     
)


terrain.Initialize(2.0, 6.0, 0.1)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(2.0, 1.5, 0.1))


terrain.EnableBulldozing(True)
terrain.EnablePlasticity(True)
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1)






vis = irr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-5, -5, 3), chrono.ChVectorD(0, 0, 0))
vis.AttachVehicle(vehicle)






driver = veh.ChIrrGuiDriver(vis)


driver.SetSteering(0.0)
driver.SetThrottle(0.3)
driver.SetBraking(0.0)






realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    driver_inputs = driver.GetInputs()
    vehicle.SetDriverInputs(driver_inputs)

    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)

    
    vis.Synchronize('', driver_inputs)
    vis.Render()

    
    vehicle.Advance(step_size)
    terrain.Advance(step_size)

    
    realtime_timer.Spin(step_size)