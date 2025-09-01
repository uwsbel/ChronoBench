import pychrono as chrono
import pychrono.irrlicht as irr

# 1. Initialize system and set gravity
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create ground body with visualization
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetName("Ground")

# Ground visualization (thick box)
ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(1, 1, 0.1)
ground.AddVisualShape(ground_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, -0.1)))
system.Add(ground)

# 3. Create mass body (box)
mass = chrono.ChBody()
mass.SetPos(chrono.ChVectorD(0, 0, 1))  # Initial position
mass.SetMass(1.0)
mass.SetName("Mass")

# Calculate proper inertia for 20cm cube
size = 0.2
inertia = (1.0/12.0) * (size**2 + size**2)
mass.SetInertiaXX(chrono.ChVectorD(inertia, inertia, inertia))

# Mass visualization
box_shape = chrono.ChBoxShape()
box_shape.GetBoxGeometry().Size = chrono.ChVectorD(size, size, size)
mass.AddVisualShape(box_shape)
system.Add(mass)

# 4. Create spring-damper link
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
spring.SetSpringCoefficient(10)    # N/m
spring.SetDampingCoefficient(1)    # N·s/m
spring.SetRestLength(1.0)          # Initial length
spring.SetName("SpringDamper")

# Spring visualization
spring_vis = chrono.ChSpringShape(0.05, 20, 20)  # radius, coils, resolution
spring.AddVisualShape(spring_vis)
system.Add(spring)

# 5. Set up Irrlicht visualization
application = irr.ChIrrApp(system, "Mass-Spring-Damper System", irr.dimension2du(800, 600))
application.AddTypicalCamera(irr.vector3df(0, 3, 2), irr.vector3df(0, 0, 0.5))
application.AddTypicalLights()
application.AddShadowAll()
application.AssetBindAll()
application.AssetUpdateAll()

# 6. Configure simulation parameters
application.SetTimestep(0.01)
application.SetTryRealtime(True)

# 7. Run simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()