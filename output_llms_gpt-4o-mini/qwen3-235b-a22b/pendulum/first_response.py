import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity in the negative Y direction

# Create the ground body (fixed)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)

# Create the pendulum body
pendulum = chrono.ChBody()
pendulum.SetMass(1.0)  # Mass of 1 kg
pendulum.SetInertiaXX(chrono.ChVectorD(0.333, 0.333, 0.333))  # Moment of inertia for a rod (1/3)*m*L^2 for L=1m

# Set initial position and rotation (45 degrees from vertical)
theta = math.radians(45)
pendulum.SetPos(chrono.ChVectorD(0, -0.5, 0))  # Center of mass at (0, -L/2, 0) assuming length L=1m
pendulum.SetRot(chrono.ChQuaternionD(math.cos(theta/2), 0, 0, math.sin(theta/2)))  # Rotate about Z-axis

sys.Add(pendulum)

# Create a revolute joint to connect the ground and pendulum
revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(
    ground, 
    pendulum, 
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0))  # Joint at origin, Z-axis rotation
)
sys.AddLink(revolute_joint)

# Set up Irrlicht visualization
app = chronoirr.ChIrrApp(sys, 'Pendulum Simulation', chronoirr.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(0, 0, 3), chronoirr.vector3df(0, -0.5, 0))  # Camera setup
app.AssetBindAll()
app.AssetUpdateAll()

# Logging configuration
log_interval = 0.1
next_log_time = 0.0

# Simulation loop
while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    sys.DoStepDynamics(1e-3)  # Step simulation with 1ms timestep

    # Log pendulum position and velocity periodically
    current_time = sys.GetChTime()
    if current_time >= next_log_time:
        pos = pendulum.GetPos()
        vel = pendulum.GetPos_dt()
        print(f"Time: {current_time:.3f}, Position: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), "
              f"Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f})")
        next_log_time += log_interval