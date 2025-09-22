import chrono as ch
import chrono.irrlicht as irr

# Initialize PyChrono environment
ch.CHGlobals.Initialize()

# Create the Irrlicht application
app = irr.IrrlichtApplication()

# Create the simulation system
system = ch.ChSystem()

# Create the ground body
ground = ch.ChBodyEasyBox(1, 1, 1, 1000, False, False)
ground.SetPos(ch.ChVector3d(0, -5, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the pendulum body
pendulum = ch.ChBodyEasyBox(0.1, 1, 0.1, 1, False, False)
pendulum.SetPos(ch.ChVector3d(0, 0, 0))
pendulum.SetRot(ch.ChQuaterniond(1, 0, 0, 0))
system.Add(pendulum)

# Define pendulum's inertia (optional, but recommended for accuracy)
# For a box of dimensions 0.1x1x0.1 and mass 1, we can use:
# pendulum.SetInertiaXX(ch.ChVector3d(0.0833, 0.0033, 0.0833))

# Create a revolute joint
joint = ch.ChRevoluteJoint(ground, pendulum, ch.ChVector3d(0, -5, 0), ch.ChVector3d(0, 0, 0))
joint.SetModulusOfElasticity(1e6)
joint.SetDissipation(0.5)
system.Add(joint)

# Add a motor (optional, for actuating the joint)
# motor = ch.ChLinkMotorRotationAngleAxis(joint)
# motor.SetAngleFunction(ch.ChFunction_Const(45), 0.01)
# system.Add(motor)

# Visualization
vis = irr.IrrlichtSystem(system, app)
vis.SetCameraVertical(chrono.ChVector3d(0, -10, 0))
vis.SetCameraTarget(chrono.ChVector3d(0, 0, 0))

# Simulation loop
while vis.Run() and not app.HasToExit():
    system.DoStepDynamics(0.02)

    # Optionally, log pendulum's position and velocity
    pendulum_pos = pendulum.GetPos()
    pendulum_vel = pendulum.GetPos_dt()
    print(f"Pendulum Position: {pendulum_pos}, Velocity: {pendulum_vel}")

# Cleanup
del vis
del app