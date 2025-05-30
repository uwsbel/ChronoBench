import pychrono as chrono
import pychrono.postprocess as postprocess
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
# Create the PyChrono system and simulation environment
# ---------------------------------------------------------------------

# Create a Chrono system
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create the ground body and the pendulum bob
# ---------------------------------------------------------------------

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 0.2, 10, 1000, True, True)
ground.SetBodyFixed(True)
system.Add(ground)

# Create the pendulum bob
pendulum_bob = chrono.ChBodyEasySphere(0.2, 1, True, True)
pendulum_bob.SetPos(chrono.ChVectorD(0, -1, 0))
system.Add(pendulum_bob)

# ---------------------------------------------------------------------
# Connect the pendulum bob to the ground using a revolute joint
# ---------------------------------------------------------------------

# Create a revolute joint between the ground and pendulum bob
revolute_joint = chrono.ChLinkLockRevolute()
revolute_joint.Initialize(ground, pendulum_bob, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(revolute_joint)

# ---------------------------------------------------------------------
# Set up the visualization system
# ---------------------------------------------------------------------

# Create the Irrlicht application
application = chronoirr.ChIrrApp(system, "Simple Pendulum Simulation", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(2, 2, 2))

# ---------------------------------------------------------------------
# Set up a data logger to record the pendulum's position and velocity
# ---------------------------------------------------------------------

# Create a data logger
data_logger = chrono.ChDataLoggerCSV("pendulum_data.csv")
data_logger.AddValue("time", lambda: system.GetChTime())
data_logger.AddValue("position_x", lambda: pendulum_bob.GetPos().x)
data_logger.AddValue("position_y", lambda: pendulum_bob.GetPos().y)
data_logger.AddValue("velocity_x", lambda: pendulum_bob.GetPos_dt().x)
data_logger.AddValue("velocity_y", lambda: pendulum_bob.GetPos_dt().y)
system.SetChDataLogger(data_logger)

# ---------------------------------------------------------------------
# Run the simulation
# ---------------------------------------------------------------------

# Set the time step
time_step = 1e-3

# Set the simulation duration
simulation_duration = 10

# Run the simulation
application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(time_step)

while (application.GetDevice().run()) and (system.GetChTime() < simulation_duration):
    system.DoStepDynamics(time_step)
    application.BeginScene()
    application.DrawAll()
    application.EndScene()

# Close the data logger
data_logger.Close()