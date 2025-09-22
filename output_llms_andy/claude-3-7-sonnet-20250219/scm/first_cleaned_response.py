import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np
import os


data_dir = os.path.join(os.path.dirname(veh.__file__), 'data')
hmmwv_data_dir = os.path.join(data_dir, 'vehicle', 'hmmwv')




chrono.SetChronoDataPath(data_dir)
mysystem = chrono.ChSystemSMC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


mysystem.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
mysystem.SetSolverMaxIterations(150)
mysystem.SetMaxPenetrationRecoverySpeed(4.0)





terrain_length = 100.0  
terrain_width = 100.0   


terrain = veh.SCMDeformableTerrain(mysystem)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(-chrono.CH_C_PI_2)))
terrain.Initialize(terrain_length, terrain_width, 0.2)


terrain.SetSoilParameters(
    2e6,      
    0,        
    1.1,      
    0,        
    30,       
    0.01,     
    4e7,      
    3e4,      
)


terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1)
terrain.SetMeshResolution(0.2)


terrain.AddMovingPatch(
    chrono.ChVectorD(0, 0, 0),  
    chrono.ChVectorD(5, 5, 0),  
    chrono.ChVectorD(0, 0, 0)   
)





initLoc = chrono.ChVectorD(0, 0.6, 0)
initRot = chrono.Q_from_AngZ(0)


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(1e-3)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


vehicle_system = hmmwv.GetSystem()
terrain.SetupMovingPatch(hmmwv.GetChassisBody(), chrono.ChVectorD(0, 0, 0), 
                         chrono.ChVectorD(5, 5, 1))


mysystem.Add(vehicle_system)





vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, -6), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()





driver = veh.ChIrrGuiDriver(vis)


steering_gain = 0.5
throttle_gain = 1.0
braking_gain = 0.8

driver.SetSteeringDelta(1.0 * steering_gain)
driver.SetThrottleDelta(1.0 * throttle_gain)
driver.SetBrakingDelta(1.0 * braking_gain)


driver.SetUserPosition(chrono.ChVectorD(0, 1.5, -6))
driver.SetUserLookAt(chrono.ChVectorD(0, 0, 0))


driver.Initialize()





t_end = 1000  
step_size = 1e-3  
render_step_size = 1.0 / 50  


mysystem.Setup()
mysystem.Update()


realtime_timer = chrono.ChRealtimeStepTimer()


time = 0.0
step_number = 0
render_steps = 0
render_frame = 0

print("SIMULATION STARTED")

while vis.Run() and time < t_end:
    
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    
    
    mysystem.DoStepDynamics(step_size)
    
    
    time += step_size
    step_number += 1
    render_steps += 1
    
    
    if render_steps >= render_step_size / step_size:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_steps = 0
        render_frame += 1
        
        
        if render_frame % 100 == 0:
            chassis_pos = hmmwv.GetVehicle().GetChassisPos()
            print(f"Time: {time:.2f} | Position: ({chassis_pos.x:.2f}, {chassis_pos.y:.2f}, {chassis_pos.z:.2f})")
    
    
    realtime_timer.Spin(step_size)

print("SIMULATION ENDED")