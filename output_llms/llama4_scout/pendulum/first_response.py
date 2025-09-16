import chrono as ch
import chrono.irrlicht as irr

# Initialize the PyChrono environment
ch.ChEngine.Initialize()

# Create the Irrlicht application
app = irr.ChIrrApp()

# Create the ground body
ground = ch.ChBody()
ground.SetPos(ch.ChVector3d(0, 0, -10))
ground.SetFixed(True)

# Create the pendulum body
pendulum = ch.ChBody()
pendulum.SetMass(1.0)
pendulum.SetPos(ch.ChVector3d(5, 0, 0))
pendulum.SetRot(ch.ChQuaterniond(1, 0, 0, 0))

# Define inertia for the pendulum
inertia = ch.ChMatrix33d(1, 0, 0, 0, 1, 0, 0, 0, 1)
inertia(0, 0) = 0.1
inertia(1, 1) = 0.1
inertia(2, 2) = 0.1
pendulum.SetInertia(inertia)

# Create a revolute joint
joint = ch.ChRevolute()
joint.Init(ground, pendulum, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))

# Create a motor (optional, for actuating the pendulum)
# motor = ch.ChFunction()
# motor = ch.ChSinCosEasePow(0, 1, 0.1, 10)
# joint.SetMotorFunction(motor)

# Create the Irrlicht scene
scene = irr.ChIrrScene()
scene.AddBody(ground)
scene.AddBody(pendulum)
scene.AddLink(joint)

# Set visualization parameters
scene.SetCameraPos(ch.ChVector3d(0, -20, 10))
scene.SetCameraTarget(ch.ChVector3d(0, 0, 0))

# Simulation loop
time = 0
dt = 0.02
while time < 30 and app.Run():
    # Update the simulation
    pendulum.Update()
    joint.Update()
    
    # Log pendulum's position and velocity
    print(f"Time: {time:.2f}, Pendulum Position: {pendulum.GetPos()}, Pendulum Velocity: {pendulum.GetPos_dt()}")
    
    # Increment time
    time += dt

    # Render the scene
    scene.Render()