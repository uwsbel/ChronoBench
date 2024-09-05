import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono environment
chrono.ChSystemSMC()

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono.GetSystem())
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Simple Pendulum Simulation')
vis.Initialize()

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 10, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -5, 0))
chrono.GetSystem().Add(ground)

# Create the pendulum body
pendulum_mass = 1
pendulum_length = 1
pendulum_inertia = chrono.ChVectorD(pendulum_mass * pendulum_length * pendulum_length / 12, 
                                    pendulum_mass * pendulum_length * pendulum_length / 12, 
                                    pendulum_mass * pendulum_length * pendulum_length / 12)
pendulum = chrono.ChBodyEasySphere(pendulum_mass, pendulum_length, True, True)
pendulum.SetInertia(pendulum_inertia)
pendulum.SetPos(chrono.ChVectorD(0, 0, 0))
chrono.GetSystem().Add(pendulum)

# Connect the pendulum and ground with a revolute joint
revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(pendulum, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.VECT_X)))
chrono.GetSystem().Add(revolute_joint)

# Set the initial position and velocity of the pendulum
pendulum.SetPos(chrono.ChVectorD(0, pendulum_length, 0))
pendulum.SetWvel_par(chrono.ChVectorD(0, 0, 0))

# Set the gravity force
chrono.GetSystem().Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Simulation settings
step_size = 0.01
simulation_time = 10

# Run the simulation
while vis.Run() and chrono.GetSystem().GetChTime() < simulation_time:
    chrono.GetSystem().DoStepDynamics(step_size)

    # Log the pendulum's position and velocity
    pos = pendulum.GetPos()
    vel = pendulum.GetPos_dt()
    print(f"Time: {chrono.GetSystem().GetChTime():.2f} | Position: {pos} | Velocity: {vel}")

    # Render the simulation
    vis.DrawAll()