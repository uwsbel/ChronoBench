import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle = veh.ARTcar()


init_pos = chrono.ChVectorD(0, 0.5, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)


vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1e-3)


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


vehicle.Initialize()


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


system = vehicle.GetSystem()


terrain = veh.RigidTerrain(system)


terrain_length = 100.0
terrain_width = 100.0
terrain_height = 0.0


patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrain_height), 
                                         chrono.QUNIT),
                        terrain_length, terrain_width)


patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


terrain.Initialize()


driver = veh.ChInteractiveDriverIRR()


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   

driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('ARTcar Vehicle Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, -8))
vis.AddTypicalLights()


driver.Initialize()


step_size = 1e-3
render_step_size = 1.0/50  
render_steps = int(render_step_size / step_size)


time = 0
step_number = 0
render_frame = 0

print("Starting simulation...")
print("Use WASD keys to control the vehicle:")
print("W - Throttle")
print("S - Brake") 
print("A - Steer Left")
print("D - Steer Right")

while vis.Run():
    time = system.GetChTime()
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    
    driver_inputs = driver.GetInputs()
    
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    
    
    system.DoStepDynamics(step_size)
    
    step_number += 1
    
    
    if step_number % 1000 == 0:
        vehicle_pos = vehicle.GetPos()
        vehicle_speed = vehicle.GetSpeed()
        print(f"Time: {time:.2f}s, Position: ({vehicle_pos.x:.2f}, {vehicle_pos.y:.2f}, {vehicle_pos.z:.2f}), Speed: {vehicle_speed:.2f} m/s")

print("Simulation completed.")