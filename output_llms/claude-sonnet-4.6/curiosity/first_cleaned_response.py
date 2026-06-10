import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.robot as robot
import math




step_size = 1e-3          
total_time = 30.0         
render_step_size = 1.0 / 50  


terrain_length = 20.0
terrain_width = 20.0


init_pos = chrono.ChVectorD(0, 0.2, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetSolverForceTolerance(1e-10)


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)





ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)


ground = chrono.ChBodyEasyBox(
    terrain_length, 0.5, terrain_width,  
    1000,                                  
    True,                                  
    True,                                  
    ground_mat
)
ground.SetPos(chrono.ChVectorD(0, -0.25, 0))
ground.SetBodyFixed(True)
ground.GetName()
ground.SetName("Ground")


ground_visual = ground.GetVisualShape(0)
if ground_visual:
    ground_texture = chrono.ChVisualMaterial()
    ground_texture.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground_texture.SetSpecularColor(chrono.ChColor(0.2, 0.2, 0.2))
    ground_visual.AddMaterial(ground_texture)
    ground_visual.SetTextureScale(4.0, 4.0)

system.Add(ground)




rock_mat = chrono.ChMaterialSurfaceNSC()
rock_mat.SetFriction(0.8)
rock_mat.SetRestitution(0.02)

rock_positions = [
    chrono.ChVectorD(3, 0.15, 1),
    chrono.ChVectorD(-2, 0.1, 3),
    chrono.ChVectorD(5, 0.12, -2),
    chrono.ChVectorD(-4, 0.18, -1),
    chrono.ChVectorD(1, 0.1, -4),
]
rock_sizes = [0.3, 0.2, 0.25, 0.35, 0.15]

for i, (pos, size) in enumerate(zip(rock_positions, rock_sizes)):
    rock = chrono.ChBodyEasySphere(size, 2500, True, True, rock_mat)
    rock.SetPos(pos)
    rock.SetBodyFixed(True)
    rock.SetName(f"Rock_{i}")
    
    rock_visual = rock.GetVisualShape(0)
    if rock_visual:
        rock_mat_vis = chrono.ChVisualMaterial()
        rock_mat_vis.SetKdTexture(chrono.GetChronoDataFile("textures/rock.jpg"))
        rock_visual.AddMaterial(rock_mat_vis)
    
    system.Add(rock)





curiosity = robot.Curiosity(system)


driver = robot.CuriositySpeedDriver(1.0, 1.0)  
curiosity.SetDriver(driver)


curiosity.Initialize(chrono.ChFrameD(init_pos, init_rot))

print("Curiosity rover initialized successfully!")
print(f"Rover position: {curiosity.GetChassisPos()}")




vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("Curiosity Rover on Rigid Terrain - PyChrono")
vis.SetWindowSize(1280, 720)


vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVectorD(-5, 3, -5),  
    chrono.ChVectorD(0, 0, 0)      
)
vis.SetCameraVertical(chrono.CameraVerticalDir_Y)


vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVectorD(10, 20, 10),   
    chrono.ChVectorD(0, 0, 0),       
    40,                               
    1, 60,                            
    50                                
)
vis.EnableShadows()


vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))




steering_angle = 0.0          
target_speed = 1.5            
time = 0.0
render_steps = int(render_step_size / step_size)
step_number = 0

print("\n=== Simulation Controls ===")
print("The rover will automatically navigate with pre-programmed maneuvers.")
print("Watch the Curiosity rover traverse the terrain!\n")




def get_steering_and_speed(t):
    
    if t < 3.0:
        
        return 0.0, 1.5
    elif t < 6.0:
        
        return 0.3, 1.2
    elif t < 9.0:
        
        return 0.0, 1.5
    elif t < 12.0:
        
        return -0.4, 1.0
    elif t < 15.0:
        
        return 0.0, 1.5
    elif t < 18.0:
        
        return 0.25, 1.2
    elif t < 21.0:
        
        return -0.25, 1.2
    elif t < 24.0:
        
        return 0.5, 0.8
    else:
        
        return 0.0, 1.5




print("Starting simulation...")

while vis.Run() and time < total_time:
    
    
    steering, speed = get_steering_and_speed(time)
    
    
    driver.SetSteering(steering)
    
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        
        
        irr.drawAllContactPoints(vis, 1.5, irr.IrrContactsDrawMode_CONTACT_DISTANCES)
        
        vis.EndScene()
        
        
        if step_number % (render_steps * 100) == 0:
            rover_pos = curiosity.GetChassisPos()
            print(f"Time: {time:.2f}s | "
                  f"Pos: ({rover_pos.x:.2f}, {rover_pos.y:.2f}, {rover_pos.z:.2f}) | "
                  f"Steering: {steering:.2f} | Speed: {speed:.2f}")
    
    
    curiosity.Update()
    
    
    system.DoStepDynamics(step_size)
    
    time += step_size
    step_number += 1

print("\nSimulation completed!")
print(f"Final rover position: {curiosity.GetChassisPos()}")