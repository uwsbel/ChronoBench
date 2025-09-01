Here is the code that meets the specifications:
```
import chrono as c
import chrono_irrlicht as irr

# Initialize the PyChrono environment
c.init()

# Create a ground body
ground = c.Body(c.ChronoMaterial(0.0, 0.0, 0.0), c.ChronoBodyData(0.0, 0.0, 0.0))
ground.SetPos(c.Vec3(0.0, 0.0, 0.0))

# Create a pendulum body
pendulum = c.Body(c.ChronoMaterial(1.0, 0.0, 0.0), c.ChronoBodyData(0.0, 0.0, 0.0))
pendulum.SetPos(c.Vec3(0.0, 1.0, 0.0))
pendulum.SetMass(1.0)
pendulum.SetInertia(c.Mat3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))

# Create a revolute joint
joint = c.RevoluteJoint(pendulum, ground, c.Vec3(0.0, 0.0, 0.0), c.Vec3(0.0, 0.0, 0.0))

# Set default parameters
pendulum.SetForce(c.Vec3(0.0, 0.0, 0.0))
pendulum.SetTorque(c.Vec3(0.0, 0.0, 0.0))

# Create a visualization system
vis = irr.IrrlichtVis()

# Set up the visualization
vis.AddBody(ground)
vis.AddBody(pendulum)
vis.AddJoint(joint)

# Set up the simulation
sim = c.Simulation()
sim.AddBody(ground)
sim.AddBody(pendulum)
sim.AddJoint(joint)

# Set up the logging
log = c.Logger()
log.AddBody(pendulum)

# Run the simulation
while True:
    sim.Step()
    vis.Render()
    log.Log()
    c.wait(0.01)
```
This code initializes the PyChrono environment, creates a ground body and a pendulum body, connects them with a revolute joint, sets default parameters, creates a visualization system, sets up the visualization, sets up the simulation, sets up the logging, and runs the simulation.