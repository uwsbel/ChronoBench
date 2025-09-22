import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math
import os




chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data/'))


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)


terrain_height = 0
terrain_length = 100.0  
terrain_width = 100.0   
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrain_height), chrono.QUNIT),
                         terrain_length, terrain_width)
                         

patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
terrain.Initialize()





viper_file = veh.GetDataFile("viper/viper.json")
rover_tire_file = veh.GetDataFile("viper/viper_tire.json")
powertrain_file = veh.GetDataFile("viper/viper_powertrain.json")


viper = veh.Viper(system)
viper.SetContactMethod(chrono.ChContactMethod_NSC)
viper.SetChassisFixed(False)
viper.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
viper.SetTireType(veh.TireModelType_RIGID)
viper.SetTireStepSize(1e-3)
viper.Initialize()


viper.SetChassisVisualizationType(veh.VisualizationType_MESH)
viper.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
viper.SetWheelVisualizationType(veh.VisualizationType_MESH)
viper.SetTireVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(viper.GetVehicle())
driver.Initialize()




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 5.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)




step_size = 1e-3  
render_step_size = 1.0 / 50  
total_simulation_time = 20.0  




time = 0
render_time = 0

steering_target = 0.0
throttle_target = 0.3  

steering_transition_time = 5.0  
max_steering = 0.3  

print("Simulation running...")

while vis.Run() and time < total_simulation_time:
    
    if time < steering_transition_time:
        
        steering_target = max_steering * (time / steering_transition_time)
    elif time < 2 * steering_transition_time:
        
        steering_target = max_steering * (1 - (time - steering_transition_time) / steering_transition_time)
    elif time < 3 * steering_transition_time:
        
        steering_target = 0
    
    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = steering_target
    driver_inputs.m_throttle = throttle_target
    driver_inputs.m_braking = 0.0
    
    
    viper.Synchronize(time, driver_inputs, terrain)
    
    
    terrain.Synchronize(time)
    system.DoStepDynamics(step_size)
    
    
    render_time += step_size
    if render_time >= render_step_size:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_time = 0
    
    
    time += step_size

print("Simulation completed.")