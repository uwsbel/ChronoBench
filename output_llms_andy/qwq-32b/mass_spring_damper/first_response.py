import pychrono as chrono
import pychrono.irrlicht as irr

# Create the Chrono system
my_system = chrono.ChSystemNSC()

# Create the Irrlicht visualization
my_application = irr.ChIrrApp(my_system, 'Mass-Spring-Damper Demo', irr.dimension2du(800, 600))
my_application.AddTypicalLogo()
my_application.AddTypicalSky()
my_application.AddTypicalLights()
my_application.SetCameraPosition(chrono.ChVectorD(3, 3, 3))
my_application.SetCameraTarget(chrono.ChVectorD(1, 0, 0))

# Create ground body (fixed)
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
my_system.Add(ground)

# Add visual shape to ground (small cube)
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.1, 0.1, 0.1))
ground_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddAsset(ground_shape)

# Create mass body
mass = chrono.ChBody()
mass.SetMass(1)  # 1 kg mass
mass.SetPos(chrono.ChVectorD(1, 0, 0))  # Initial position 1m from ground
my_system.Add(mass)

# Add visual shape to mass (larger cube)
mass_shape = chrono.ChBoxShape()
mass_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.2, 0.2, 0.2))
mass_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.3))
mass.AddAsset(mass_shape)

# Create spring-damper link using ChLinkTSDA
springdamper = chrono.ChLinkTSDA()
springdamper.Initialize(
    ground, mass,  # Connect ground to mass
    chrono.ChFrameD(),  # Attachment point on ground (local origin)
    chrono.ChFrameD()   # Attachment point on mass (local origin)
)
springdamper.SetSpringK(100)      # Spring constant (N/m)
springdamper.SetSpringRestLength(0)  # Rest length (m)
springdamper.SetDampingR(10)      # Damping coefficient (Ns/m)
my_system.AddLink(springdamper)

# Add spring visualization
spring_shape = irr.ChSpringShape(springdamper, 10, 0.1)  # 10 coils, 0.1m radius
springdamper.AddAsset(spring_shape)

# Final setup for visualization
my_application.AssetBind()
my_application.AssetUpdate()
my_application.SetTimestep(0.005)  # Simulation time step

# Run the simulation
my_application.Run()