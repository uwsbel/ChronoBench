import pychrono as chrono

# Initialize the Chrono system
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBody()
ground.SetName("Ground")
ground.SetFixed(True)
ground.EnableCollision(False)
ground_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
ground.AddVisualShape(ground_shape, chrono.ChFramed(chrono.ChVectorD(0, 0, 0)))
sys.AddBody(ground)

# Create the pendulum body
pendulum = chrono.ChBody()
pendulum.SetName("Pendulum")
pendulum.SetMass(1.0)
pendulum.SetInertiaXX(chrono.ChVectorD(0, 1/12, 1/12))  # Ixx=0, Iyy=Izz=1/12 kg·m² for a 1m rod
pendulum.SetCOMpos(chrono.ChVectorD(0.5, 0, 0))  # Center of mass offset
cyl = chrono.ChVisualShapeCylinder(0.05, 1.0)  # Cylinder for pendulum visualization
pendulum.AddVisualShape(cyl, chrono.ChFramed(chrono.ChVectorD(0.5, 0, 0), chrono.Q_from_AngZ(-chrono.CH_PI_2)))
sys.AddBody(pendulum)

# Create the revolute joint
revolute = chrono.ChLinkLockRevolute()
revolute.Initialize(ground, pendulum, chrono.ChFramed(chrono.ChVectorD(0, 0, 0)))
sys.AddLink(revolute)

# Set up Irrlicht visualization
application = chrono.ChVisualSystemIrrlicht()
application.AttachSystem(sys)
application.SetWindowSize(1024, 720)
application.SetWindowTitle('Simple Pendulum Simulation')
application.Initialize()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
application.AddTypicalLights()

# Simulation loop parameters
time_step = 0.001
log_interval = 0.05
next_log = 0.0

# Run the simulation
while application.Run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    sys.DoStepDynamics(time_step)
    current_time = sys.GetChTime()
    if current_time >= next_log:
        com_pos = pendulum.GetPos_com()
        com_vel = pendulum.GetLinVel()
        ang_vel = pendulum.GetWvel_loc()
        print(f"Time: {current_time:.3f} s")
        print(f"COM Position: {com_pos.x:.4f}, {com_pos.y:.4f}, {com_pos.z:.4f} m")
        print(f"COM Velocity: {com_vel.x:.4f}, {com_vel.y:.4f}, {com_vel.z:.4f} m/s")
        print(f"Angular Velocity: {ang_vel.z:.4f} rad/s")
        next_log += log_interval