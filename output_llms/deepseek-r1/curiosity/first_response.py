import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import os

# Set path to Chrono data directory
chrono.SetChronoDataPath("path/to/chrono/data/")  # Update with your actual path

# Create physical system
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_PSS)
system.SetSolverMaxIterations(100)

# Create textured ground
ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.01)

ground = chrono.ChBodyEasyBox(50, 1, 50, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Add ground texture
ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(ground_texture)

# Create Curiosity rover
rover = robot.Curiosity()
rover.SetDriver(robot.CuriosityDCMotorControl())
rover.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, -0.4, 0), chrono.QUNIT))
system.Add(rover.GetChassis().GetBody())

# Initialize Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 50, 5, 20, 50, 512)

# Enable shadows
vis.EnableShadows()

# Simulation parameters
time_step = 0.001
time = 0
last_time = 0

# Driver parameters (simulate steering inputs)
steering_angle = 0.0
speed_level = 0.0

# Main simulation loop
while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs (example: periodic steering)
    steering_angle = 0.3 * chrono.ChSineStep(time, 1.0, 0.5)
    speed_level = 0.5
    
    rover.GetDriver().SetSteering(steering_angle)
    rover.GetDriver().SetSpeed(speed_level)
    rover.GetDriver().Update(time_step)
    
    # Visual updates
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance simulation
    system.DoStepDynamics(time_step)
    
    # Camera follow rover
    if time - last_time > 0.02:  # Update every 20ms
        cam_pos = rover.GetChassis().GetBody().GetPos() + chrono.ChVectorD(-3, 1, 0)
        vis.GetActiveCamera().setTarget(chronoirr.vector3df(rover.GetChassis().GetBody().GetPos().x, 
                                                          rover.GetChassis().GetBody().GetPos().y,
                                                          rover.GetChassis().GetBody().GetPos().z))
        vis.GetActiveCamera().setPosition(chronoirr.vector3df(cam_pos.x, cam_pos.y, cam_pos.z))
        last_time = time