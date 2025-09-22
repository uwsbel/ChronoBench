import pychrono as chrono
import pychrono.irrlicht as vis

# Initialize Chrono and Irrlicht
app = vis.Application(vis.ApplicationFlags.USE_MOUSE_LOOK)
vis.add_default_lights(app)
world = chrono.ChSystemNSC()

# Create ground body
ground = chrono.ChBodyEasyBox(10, 10, 1, 1000)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
world.AddBody(ground)

# Create pendulum body
pendulum_mass = 1.0
pendulum_length = 1.0
pendulum_inertia = chrono.ChVectorD(0.1, 0.1, 0.1)  # Example inertia tensor

pendulum_body = chrono.ChBodyEasySphere(pendulum_mass, pendulum_inertia)
pendulum_body.SetPos(chrono.ChVectorD(0, pendulum_length, 0))
world.AddBody(pendulum_body)

# Create revolute joint
joint = chrono.ChLinkLockRevolute(ground, pendulum_body,
                                  chrono.ChVectorD(0, 0, 0),
                                  chrono.ChVectorD(0, 0, 1))
world.AddLink(joint)

# Visualization
vis_body = vis.ChVisualSystemIrrlicht(world, app.get_device())
vis_body.SetCamera(chrono.ChVectorD(2, 2, 2))
vis_body.SetPos(chrono.ChVectorD(0, 0, 0))

# Simulation loop
dt = 0.01
while app.run():
    world.Step(dt)
    
    # Log pendulum position and velocity
    print(f"Time: {world.GetChTime()}, Pendulum Angle: {pendulum_body.GetRot().GetA1()}")

    vis_body.render()