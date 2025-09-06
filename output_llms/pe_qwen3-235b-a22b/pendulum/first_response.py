import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Create the simulation system
sys = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.EnableCollision(False)
sys.Add(ground)

# Add a visual shape to the ground (small red sphere)
ground_shape = chrono.ChVisualShapeSphere(0.05)
ground_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red
ground.AddVisualShape(ground_shape)

# Create the pendulum material (even though collision is disabled)
material = chrono.ChContactMaterialNSC()

# Create the pendulum body
pendulum = chrono.ChBodyEasyBox(0.1, 1.0, 0.1, 1000, True, False, material)
pendulum.SetPos(chrono.ChVector3d(0, -0.5, 0))  # Center at (0, -0.5, 0)
pendulum.SetRot(chrono.QuatFromAngleZ(math.pi / 4))  # Initial rotation of 45 degrees
pendulum.EnableCollision(False)
sys.Add(pendulum)

# Create the revolute joint between ground and pendulum
rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.Add(rev_joint)

# Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Pendulum Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 5))
vis.AddTypicalLights()

# Logging setup
log_interval = 0.5  # seconds between logs
next_log_time = 0.0

# Simulation loop
while vis.Run():
    current_time = sys.GetChTime()
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        ang_vel = pendulum.GetWvel_par()
        print(f"Time: {current_time:.3f} s")
        print(f"Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        print(f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        print(f"Angular Velocity: ({ang_vel.x:.3f}, {ang_vel.y:.3f}, {ang_vel.z:.3f})")
        print("-" * 40)
        next_log_time += log_interval

    # Step the simulation
    sys.DoStepDynamics(0.01)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()