import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math





chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/"))


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))





terrain = veh.RigidTerrain(system)


terrain_patch_dims = chrono.ChVectorD(100, 0.2, 100)
terrain_mat = chrono.ChMaterialSurfaceSMC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.01)

patch_params = veh.RigidTerrainGeometry.Patch(
    chrono.ChCoordsysD(chrono.ChVectorD(0, -0.1, 0), chrono.QUNIT),
    terrain_patch_dims
)
patch_params.m_material = terrain_mat


terrain_tex = veh.RigidTerrainVisualMesh(
    chrono.GetChronoDataFile("terrain/textures/dirt.jpg"),
    chrono.ChVectorD(terrain_patch_dims.x, 0, terrain_patch_dims.z),
    chrono.ChVectorD(5, 5, 5)
)
patch_params.m_visual_mesh = terrain_tex


terrain.AddPatch(patch_params)
terrain.Initialize()





gator_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                          "data/vehicle/Gator/vehicle/Gator.json")

gator = veh.WheeledVehicle(system, gator_file)


init_pos = chrono.ChVectorD(0, 0.5, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  
gator.Initialize(chrono.ChCoordsysD(init_pos, init_rot))


gator.SetChassisVisualizationType(veh.VisualizationType_MESH)
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)





powertrain_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                              "data/vehicle/Gator/powertrain/Gator_SimpleMapPowertrain.json")
powertrain = veh.SimpleMapPowertrain(powertrain_file)
gator.InitializePowertrain(powertrain)





tire_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                        "data/vehicle/Gator/tire/Gator_TMeasyTire.json")

for axle in range(gator.GetNumberOfAxles()):
    for wheel in range(2):
        tire = veh.TMeasyTire(tire_file)
        tire.SetVisualizationType(veh.VisualizationType_MESH)
        gator.InitializeTire(tire, axle, wheel, veh.WheelState_ROTATION)





driver = veh.ChIrrGuiDriver(gator.GetVehicle())
driver.SetSteeringDelta(1.0 / 50)      
driver.SetThrottleDelta(1.0 / 50)      
driver.SetBrakingDelta(1.0 / 50)       
driver.Initialize()





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle Demo")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(3.5, 0.5, 1.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(gator.GetVehicle())





realtime_timer = chrono.ChRealtimeStepTimer()
step_size = 1e-3  


time = 0
step_number = 0


t_end = 1000  
render_step_size = 1.0 / 50  
render_steps = math.ceil(render_step_size / step_size)

print("Vehicle mass: " + str(gator.GetVehicle().GetMass()))


while vis.Run() and time < t_end:
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    
    
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    
    terrain.Advance(step_size)
    gator.Advance(step_size)
    vis.Advance(step_size)
    
    
    time += step_size
    step_number += 1
    
    
    realtime_timer.Spin(step_size)



print("Simulation ended at time: " + str(time))