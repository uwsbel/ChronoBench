import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.physics as chronophys

# Initialize the PyChrono environment
chrono.SetChronoDataPath('../data/')

# Create a Chrono::Engine physical system
system = chrono.ChSystemNSC()

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.05))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the mass body
mass = chrono.ChBodyEasySphere(0.2, 1000, True, True)
mass.SetPos(chrono.ChVectorD(0, 0, 1))
system.Add(mass)

# Create the spring-damper connection
spring = chronophys.ChLinkTSDA()
spring.Initialize(mass, ground, False, chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(0, 0, 0), False)
spring.Set_Spring_coefficient(1000)
spring.Set_Damping_coefficient(10)
spring.Set_Rest_length(1)
system.Add(spring)

# Create visual shapes for the mass and spring
# Mass visual shape
mass_vis_shape = chrono.ChSphereShape()
mass_vis_shape.GetSphereGeometry().rad = 0.2
mass_vis_shape.GetSphereGeometry().pos = chrono.ChVectorD(0, 0, 0)
mass_vis_shape.SetColor(chrono.ChColor(1, 0, 0))  # Red color
mass.AddVisualShape(mass_vis_shape)

# Spring visual shape
spring_vis_shape = chrono.ChCylinderShape()
spring_vis_shape.GetCylinderGeometry().rad = 0.02
spring_vis_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, 0)
spring_vis_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 1)
spring_vis_shape.SetColor(chrono.ChColor(0, 1, 0))  # Green color
mass.AddVisualShape(spring_vis_shape)

# Create the Irrlicht visualization system
visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Mass-Spring-Damper System')
visualization.SetTryRealTimeRate(30)
visualization.SetCameraVertical(chrono.ChVectorD(0, 1, 0))
visualization.SetCameraTarget(chrono.ChVectorD(0, 0, 0.5))
visualization.SetCameraDistance(2)

# Set up the lighting
visualization.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0), 1, 1, 1, 1, 50)

# Run the simulation
while visualization.Run():
    system.DoStepDynamics(0.01)

# Clean up
visualization.Close()