import pychrono as chrono
import pychrono.irrlicht as chronoirrl

def main():
    # Create the Chrono system
    my_system = chrono.ChSystemNSC()

    # Create ground body (pivot point)
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    # Add a small sphere for visualization
    ground_shape = chrono.ChSphereShape()
    ground_shape.GetSphereGeometry().rad = 0.05
    ground.AddAsset(ground_shape)
    ground.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.5, 0.5, 0.5)))  # Grey color
    my_system.Add(ground)

    # Create pendulum body
    pendulum_radius = 0.1  # meters
    pendulum_mass = 1.0    # kg
    # Calculate density to achieve desired mass
    density = pendulum_mass / ( (4/3)*chrono.CH_C_PI*(pendulum_radius**3) )
    pendulum = chrono.ChBodyEasySphere(pendulum_radius, density, True, True)
    pendulum.SetPos(chrono.ChVectorD(0.5, 0, -chrono.sqrt(3)/2))  # Displaced initial position
    pendulum.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0, 0)))  # Red color
    my_system.Add(pendulum)

    # Create revolute joint between ground and pendulum
    revolute = chrono.ChLinkLockRevolute()
    revolute.Initialize(ground, pendulum, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
    # Set rotation axis to Y-axis (allows rotation in X-Z plane)
    revolute.SetAxis(chrono.ChVectorD(0, 1, 0))
    my_system.AddLink(revolute)

    # Setup visualization
    vis = chronoirrl.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Simple Pendulum Simulation')
    vis.SetAntialiasing(0)
    vis.SetCameraPosition(chrono.ChVectorD(0, 3, 3))
    vis.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI/4, chrono.ChVectorD(1,0,0)))
    my_system.SetVisualSystem(vis)

    # Initialize the simulation
    my_system.SetupSimulation()
    my_system.DoStepDynamics(0)  # Initialize

    # Start visualization
    vis.BEGIN()

    # Simulation loop
    step = 0
    while vis.Run() and my_system.GetChTime() < 10:
        my_system.DoStepDynamics(0.005)  # Time step of 5ms

        # Log data every 10 steps (~0.05 seconds)
        if step % 10 == 0:
            pos = pendulum.GetPos()
            vel = pendulum.GetPos_dt()  # Linear velocity
            print(f"Time: {my_system.GetChTime():.2f}s | Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) | Velocity: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})")

        vis.Render()
        step += 1

    vis.END()

if __name__ == '__main__':
    main()