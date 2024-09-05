import pychrono as ch
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


time_step = 1e-3


vehicle = veh.HMMWV_Full()  
vehicle.SetContactMethod(ch.ChContactMethod_NSC)  
vehicle.SetChassisFixed(False)  
vehicle.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 0.5), ch.QUNIT))  
vehicle.SetTireType(veh.TireModelType_RIGID)  
vehicle.SetTireStep(veh.TireStepType_RK4)  


vehicle.Initialize()


vis_type = veh.VisualizationType_MESH
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().AddAsset(ch.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("hmmwv/hmmwv_chassis.obj"), True, True))
vehicle.GetSystem().AddAsset(ch.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("hmmwv/hmmwv_wheel.obj"), True, True))
vehicle.GetSystem().AddAsset(ch.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("hmmwv/hmmwv_tire.obj"), True, True))


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6,  
                          0,    
                          1.2,  
                          0,    
                          30    
                          )
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)  
terrain.SetMeshWidth(2, 2)  
terrain.SetMovingPatch(True, 20, 20)  
terrain.Initialize(veh.GetDataFile("terrain/terrain2.dat"), 0.02, 0.02)  


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(veh.CameraOffset(3.5, 0, 1.2), 0.5, 0.5)
vis.Initialize()
vis.AddLogo(veh.GetDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(rendering_time)
driver.SetThrottleDelta(rendering_time)
driver.SetBrakingDelta(rendering_time)
driver.Initialize()


run_time = 200.0


realtime_timer = ch.ChRealtimeStepTimer()
step_number = 0
rendering_step_number = 0


rendering_steps = math.ceil(1.0 / time_step)


terrain.InitializePatch(0, 0)


while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % rendering_steps == 0 :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        rendering_step_number = 0

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1
    rendering_step_number += 1

    
    realtime_timer.Spin(step_size)