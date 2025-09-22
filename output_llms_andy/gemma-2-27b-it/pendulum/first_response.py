import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
#
#  Create the simulation system and add the ground body
#
# ---------------------------------------------------------------------

my_system = chrono.ChSystemNSC()

# Set gravitational acceleration
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -2, 0))
my_system.Add(ground)

# ---------------------------------------------------------------------
#
#  Create the pendulum body
#
# ---------------------------------------------------------------------

pendulum_length = 1.0
pendulum_mass = 1.0

pendulum = chrono.ChBodyEasySphere(0.2, 1000, True, True)
pendulum.SetPos(chrono.ChVectorD(0, pendulum_length, 0))

# Calculate inertia for the sphere
inertia = chrono.ChVectorD(
    0.4 * pendulum_mass * 0.2**2, 0.4 * pendulum_mass * 0.2**2, 0.4 * pendulum_mass * 0.2**2
)
pendulum.SetInertia(inertia)

my_system.Add(pendulum)

# ---------------------------------------------------------------------
#
#  Create a revolute joint between the ground and pendulum
#
# ---------------------------------------------------------------------

joint = chrono.ChLinkRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
my_system.Add(joint)

# ---------------------------------------------------------------------
#
#  Create the Irrlicht visualization system
#
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Simple Pendulum Simulation")
vis.Initialize()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(5, 5, 5))

# ---------------------------------------------------------------------
#
#  Simulation loop
#
# ---------------------------------------------------------------------

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    my_system.DoStepDynamics(0.01)

    # Log pendulum position and velocity
    print(f"Pendulum position: {pendulum.GetPos()}")
    print(f"Pendulum velocity: {pendulum.GetLinVel()}")