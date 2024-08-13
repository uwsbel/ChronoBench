# Import necessary modules
from pychrono import *
from pychrono.irrlicht import *
from pychrono.fea import *

# Initialize the PyChrono environment
chrono = ChronoEngine()

# Initialize the Irrlicht visualization system
vis = ChronoVisualSystemIrrlicht(chrono, "Simple Pendulum Simulation")
vis.Initialize(pyrr::vector3d(800, 600, 0))
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(pyrr::vector3d(0, 5, -15), pyrr::vector3d(0, 0, 0))

# Create ground body
ground = chrono.AddBody(RigidBody())
ground.SetCollisionType(CollisionType.CT_RIGID)
ground_shape = BoxShape(1, 0.1, 1)
ground.AddShape(ground_shape)
ground.SetPos(pyrr::vector3d(0, -1, 0))

# Create pendulum body
pendulum_length = 1.0
pendulum_mass = 1.0
pendulum_radius = 0.1
pendulum_body = chrono.AddBody(RigidBody())
pendulum_body.SetCollisionType(CollisionType.CT_RIGID)

# Add pendulum shape
pendulum_shape = CylinderShape(pendulum_radius, pendulum_length)
pendulum_body.AddShape(pendulum_shape)

# Set pendulum mass and inertia
pendulum_body.SetMass(pendulum_mass)
inertia = pendulum_shape.ComputeLocalInertia(pendulum_mass)
pendulum_body.SetInertiaXX(inertia)

# Set pendulum initial position
pendulum_body.SetPos(pyrr::vector3d(0, 1, 0))

# Create revolute joint
joint = chrono.AddJoint(RevoluteJoint)
joint.SetBody1(ground)
joint.SetBody2(pendulum_body)
joint.SetAnchor1(pyrr::vector3d(0, 0, 0))
joint.SetAnchor2(pyrr::vector3d(0, 1, 0))

# Set the simulation parameters
chrono.SetDefaultSolver(ChSystemSolver.SR_RK4)
chrono.SetSolverType(ChSolver.Type.SR_RK4)
chrono.SetStepSize(1e-3)
chrono.SetMaxIterations(100)

# Set up logging
log = DataLogger()
log.SetTimeInterval(0.01)
log.AddColumn("time", DataLogger.Type.REAL)
log.AddColumn("position", DataLogger.Type.VECTOR3)
log.AddColumn("velocity", DataLogger.Type.VECTOR3)

# Function to be called periodically
def LogData(time, system):
    pendulum_pos = pendulum_body.GetPos()
    pendulum_vel = pendulum_body.GetVel()
    log.AddRow([time, pendulum_pos, pendulum_vel])

# Run the simulation
sim_time = 5.0
while chrono.GetChTime() < sim_time:
    chrono.DoStepDynamics()
    if chrono.GetChTime() % 0.01 == 0:
        LogData(chrono.GetChTime(), chrono)

    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

# Save the log data
log.WriteToFile("pendulum_log.csv")

# Clean up
vis.Close()
chrono.Clear()