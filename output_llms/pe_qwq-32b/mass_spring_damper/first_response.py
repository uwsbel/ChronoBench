import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    # Initialize physical system with gravity
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity vector

    # Initialize visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Mass-Spring-Damper Simulation')
    vis.Initialize()
    vis.AddCamera(chronoirr.ChVectorD(0, 2, 2))  # Camera position
    vis.AddTypicalLights()

    # Create ground body (fixed)
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    # Add visual shape to ground (small cube)
    ground_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
    ground.AddVisualShape(ground_shape)
    sys.Add(ground)

    # Create mass body (movable)
    mass = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True)
    mass.SetPos(chrono.ChVectorD(0, 0, 0.5))
    sys.Add(mass)

    # Create spring-damper link between ground and mass
    springdamper = chrono.ChLinkTSDA()
    springdamper.Initialize(ground, mass, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
    springdamper.SetSpringK(100)      # Spring stiffness (N/m)
    springdamper.SetDampingR(10)     # Damping coefficient (Ns/m)
    springdamper.SetRestLength(0.5)  # Rest length (m)
    sys.AddLink(springdamper)

    # Create visual shape for the spring (cylinder)
    spring_shape = chrono.ChVisualShapeCylinder(0.02, 0.5)  # Radius 0.02m, rest length 0.5m
    spring_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red color
    sys.AddVisualShape(spring_shape)  # Add as standalone visual shape

    # Simulation time step
    time_step = 0.001

    # Main simulation loop
    while vis.Run():
        vis.BeginScene()
        
        # Update spring visual position, orientation, and scale
        ground_pos = ground.GetPos()
        mass_pos = mass.GetPos()
        delta = mass_pos - ground_pos
        length = delta.Length()
        axis = delta / length if length > 0 else chrono.ChVectorD(0,0,1)
        midpoint = (ground_pos + mass_pos) * 0.5
        
        # Calculate rotation to align cylinder with spring direction
        rotation = chrono.ChQuaternionD()
        rotation.QlookAtDir(delta, chrono.ChVectorD(0, 1, 0))  # Up vector along Y-axis
        
        # Update visual properties of the spring
        spring_shape.SetPos(midpoint)
        spring_shape.SetRot(rotation)
        scale_factor = length / 0.5  # Scale based on current length vs rest length
        spring_shape.SetScale(chrono.ChVectorD(1, 1, scale_factor))
        
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()