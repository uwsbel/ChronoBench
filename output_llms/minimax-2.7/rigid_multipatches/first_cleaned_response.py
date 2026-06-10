import numpy as np
import math
import os


import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr






SIMULATION_STEP = 5e-4  
RENDER_STEP = 1.0 / 60.0  


VEHICLE_START_POS = chrono.ChVectorD(0, 1.0, 0)


TERRAIN_LENGTH = 100.0  
TERRAIN_WIDTH = 20.0    






my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetSolverType(chrono.ChSolver.Type_PSPEC)  
my_system.SetMaxItersSolverSpeed(100)
my_system.SetMaxItersSolverStab(100)


my_system.SetUseSleeping(True)
my_system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

print("Chrono system initialized successfully")






terrain = veh.RigidTerrain(my_system)


patch1 = terrain.AddPatch(
    chrono.ChVectorD(-50, 0, 0),      
    chrono.ChVectorD(0, 0, 0),        
    TERRAIN_WIDTH,                    
    30,                               
    0.0,                              
    veh.ContactMethod_NSC             
)
patch1.SetTexture(veh.GetDataFile("vehicle/terrain/textures/tile_asphalt.png"))
patch1.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
patch1.SetFriction(0.8)
patch1.SetRestitution(0.1)
print("Patch 1: Flat asphalt road created")


patch2 = terrain.AddPatch(
    chrono.ChVectorD(-20, 0, 0),      
    chrono.ChVectorD(0, 0, 0),
    TERRAIN_WIDTH,
    20,
    0.0,
    veh.ContactMethod_NSC
)
patch2.SetTexture(veh.GetDataFile("vehicle/terrain/textures/dirt.png"))
patch2.SetColor(chrono.ChColor(0.55, 0.35, 0.15))
patch2.SetFriction(0.6)
patch2.SetRestitution(0.05)
print("Patch 2: Dirt/gravel section created")


patch3 = terrain.AddPatch(
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 0, 0),
    TERRAIN_WIDTH,
    15,
    0.0,
    veh.ContactMethod_NSC
)
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.2))


bump_mesh = chrono.ChTriangleMeshConnected()


bump_width = TERRAIN_WIDTH / 2
bump_length = 15.0
num_x_points = 31
num_z_points = 7

for i in range(num_z_points):
    for j in range(num_x_points):
        x = -bump_length/2 + j * (bump_length / (num_x_points - 1))
        z = -bump_width + i * (bump_width / (num_z_points - 1))
        
        
        if 2.0 < x < 10.0:  
            bump_amplitude = 0.3 * math.sin((x - 2.0) * math.pi / 8.0) * \
                           (1.0 if abs(z) < bump_width * 0.8 else 0.3)
            y = bump_amplitude
        elif 10.0 < x < 13.0:  
            y = 0.15 * (x - 10.0) / 3.0
        elif 13.0 < x < 14.0:  
            y = 0.15
        else:
            y = 0.0
        
        vertex = chrono.ChVectorD(x, y, z)
        bump_mesh.AddNode(vertex)


for i in range(num_z_points - 1):
    for j in range(num_x_points - 1):
        v0 = i * num_x_points + j
        v1 = v0 + 1
        v2 = v0 + num_x_points
        v3 = v2 + 1
        
        bump_mesh.AddTriangle(v0, v1, v2)
        bump_mesh.AddTriangle(v1, v3, v2)

bump_mesh.RepairDuplicateVertices(1e-6)
patch3.SetMesh(bump_mesh, False)
print("Patch 3: Mesh-based bump terrain created")


patch4 = terrain.AddPatch(
    chrono.ChVectorD(15, 0, 0),
    chrono.ChVectorD(0, 0, 0),
    TERRAIN_WIDTH,
    30,
    0.0,
    veh.ContactMethod_NSC
)
patch4.SetColor(chrono.ChColor(0.2, 0.5, 0.2))


def height_function(x, z):
    
    
    h = 0.5 * math.sin(0.2 * x) * math.cos(0.3 * z)
    h += 0.2 * math.sin(0.5 * x + 1.0) * math.sin(0.4 * z)
    h += 0.1 * math.sin(1.0 * x) * math.cos(0.8 * z)
    h += 0.05 * math.sin(2.0 * x) * math.sin(1.5 * z)
    
    
    edge_smoothing = min(1.0, max(0.0, (x - 13.0) / 2.0))
    return h * edge_smoothing

patch4.SetHeightfieldFunction(height_function)
patch4.SetTexture(veh.GetDataFile("vehicle/terrain/textures/grass.png"))
patch4.SetFriction(0.7)
print("Patch 4: Heightmap-based hilly terrain created")


terrain.Initialize()
print("Terrain system initialized")






print("\nCreating HMMWV vehicle...")


hmmwv_params = veh.HMMWV params()
hmmwv_params.chassis_pos = VEHICLE_START_POS
hmmwv_params.chassis_fixed = False
hmmwv_params.engine_type = veh.EngineModel_Type_SHAFTS
hmmwv_params.transmission_type = veh.TransmissionModel_Type_AUTOMATIC_STEPTHROUGH
hmmwv_params.drivetrain_type = veh.DrivetrainModel_Type_SIMPLE
hmmwv_params.num_wheels = 4
hmmwv_params.chassis_vis = veh.VisualizationType_MESH
hmmwv_params.wheel_vis = veh.VisualizationType_MESH
hmmwv_params.driver_vis = veh.VisualizationType_MESH


my_hmmwv = veh.HMMWV(my_system, hmmwv_params)


my_hmmwv.Initialize(chrono.ChCoordsysD(
    VEHICLE_START_POS,
    chrono.ChQuaternionD(1, 0, 0, 0)
))


my_hmmwv.SetInitWheelAngVel(chrono.ChVectorD(5, 5, 5, 5))  

print("HMMWV vehicle initialized successfully")
print(f"  - Chassis mass: {my_hmmwv.GetChassisMass():.2f} kg")
print(f"  - Number of wheels: {my_hmmwv.GetNumWheels()}")






driver = veh.ChInteractiveDriverIRR(my_hmmwv)


steering_threshold = 0.5      
throttle_threshold = 0.8      
brake_threshold = 0.6         

driver.Initialize()
print("\nInteractive driver system initialized")
print("  Controls:")
print("    - W/Up Arrow: Accelerate")
print("    - S/Down Arrow: Brake/Reverse")
print("    - A/Left Arrow: Steer Left")
print("    - D/Right Arrow: Steer Right")
print("    - Space: Emergency Brake")
print("    - R: Reset Vehicle")
print("    - ESC: Exit Simulation")






app = chronoirr.ChChronoIRRApp(
    my_system,                    
    "HMMWV Simulation",           
    chronoirr.dimension2du(1280, 720),  
    chronoirr.VIEW_ISO,           
    True,                         
    True,                         
    chronoirr.dimension2du(0, 0)  
)


app.SetSkyBox()
app.AddTypicalLights(chronoirr.dimension2du(0, 0))
app.AddCamera(
    chronoirr.vector3df(5, 3, -5),    
    chronoirr.vector3df(0, 1, 0),     
    60,                                
    0.1,                               
    1000.0                             
)


app.AddHUDLine("HMMWV Terrain Simulation - PyChrono")
app.AddHUDLine("Controls: W/S - Throttle/Brake | A/D - Steering | Space - E-Brake | R - Reset")


app.SetSymbolDrawer(chronoirr.ChVisualizationUtils)


my_hmmwv.SetVisualization(veh.VisualizationType_MESH)


for subsystem in my_hmmwv.GetSubSystems():
    app.Add(subsystem.GetBody())


app.Add(terrain.GetGroundBody())

print("\nIrrlicht visualization initialized")





print("\n" + "="*60)
print("STARTING SIMULATION")
print("="*60)


simulation_time = 0.0
render_time = 0.0
frame_count = 0
max_simulation_time = 120.0  


def custom_step(system, step_size):
    
    
    system.DoStepDynamics(step_size)
    
    
    my_hmmwv.Synchronize(simulation_time, driver.GetInputs())
    
    
    driver.Synchronize(simulation_time)
    
    return True


app.SetTimestep(SIMULATION_STEP)

while app.GetDevice().run():
    
    if app.GetDevice().getEventReceiver().IsKeyDown(irrlicht_key_R):
        
        my_hmmwv.SetPosition(chrono.ChCoordsysD(
            VEHICLE_START_POS,
            chrono.ChQuaternionD(1, 0, 0, 0)
        ))
        my_hmmwv.SetVelocity(chrono.ChVectorD(0, 0, 0))
        my_hmmwv.SetAngularVelocity(chrono.ChVectorD(0, 0, 0))
        my_hmmwv.Initialize(chrono.ChCoordsysD(
            VEHICLE_START_POS,
            chrono.ChQuaternionD(1, 0, 0, 0)
        ))
        simulation_time = 0.0
        print("Vehicle reset!")
    
    
    
    
    if render_time >= RENDER_STEP:
        app.BeginScene(True, True, chronoirr.SColor(255, 100, 100, 100))
        app.DrawAll()
        
        
        app.DrawCaption(f"Time: {simulation_time:.2f}s | FPS: {1.0/RENDER_STEP:.0f}")
        
        app.EndScene()
        render_time = 0.0
        frame_count += 1
    
    
    app.DoStep()
    simulation_time += SIMULATION_STEP
    render_time += SIMULATION_STEP
    
    
    if frame_count % 600 == 0:  
        print(f"Progress: {simulation_time:.1f}s | Frames: {frame_count}")
    
    
    if simulation_time >= max_simulation_time:
        print(f"\nSimulation completed: Reached time limit ({max_simulation_time}s)")
        break





print("\n" + "="*60)
print("SIMULATION ENDED")
print("="*60)


print(f"\nFinal Statistics:")
print(f"  - Total simulation time: {simulation_time:.2f} seconds")
print(f"  - Total frames rendered: {frame_count}")
print(f"  - Average FPS: {frame_count / simulation_time:.1f}")


chassis_pos = my_hmmwv.GetChassisPos()
chassis_vel = my_hmmwv.GetChassisVel()
print(f"\nFinal Vehicle State:")
print(f"  - Position: ({chassis_pos.x:.2f}, {chassis_pos.y:.2f}, {chassis_pos.z:.2f})")
print(f"  - Velocity: ({chassis_vel.x:.2f}, {chassis_vel.y:.2f}, {chassis_vel.z:.2f})")

print("\nSimulation script completed successfully!")