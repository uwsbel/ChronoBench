import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # Set path to Chrono data directory

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add the required physical systems and objects

# Create a ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)  # Make it static
system.Add(ground)

# Create a mass body
mass = chrono.ChBody()
mass.SetMass(1.0)  # Set mass to 1 kg
mass.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  # Small inertia
mass.SetPos(chrono.ChVectorD(0, 1, 0))  # Initial position
system.Add(mass)

# 3. Set necessary default parameters such as positions, forces, and interactions

# Create a spring-damper between ground and mass
spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass, False,
                        chrono.ChVectorD(0, 0, 0),  # Connection point on ground
                        chrono.ChVectorD(0, 0, 0))  # Connection point on mass

# Set default spring-damper parameters
spring_damper.SetSpringCoefficient(100.0)  # Spring stiffness [N/m]
spring_damper.SetDampingCoefficient(5.0)   # Damping coefficient [N·s/m]
spring_damper.SetRestLength(1.0)          # Rest length [m]

system.Add(spring_damper)

# Create visualization assets
ground_vis = chrono.ChBoxShape()
ground_vis.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 0.1, 10))
ground_vis.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.AddAsset(ground_vis)

mass_vis = chrono.ChSphereShape()
mass_vis.GetSphereGeometry().rad = 0.2
mass.AddAsset(mass_vis)

# Create a visual representation of the spring
spring_vis = chrono.ChSpringShape()
spring_vis.SetSpringCoilDiameter(0.1)
spring_vis.SetSpringFreeLength(1.0)
spring_vis.SetSpringNaturalLength(1.0)
spring_vis.SetSpringWindings(20)
spring_damper.AddAsset(spring_vis)

# 4. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 3))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1, 5, -2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)

# Simulation loop
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation by one step
    system.DoStepDynamics(time_step)

    # Get and print the position of the mass
    mass_pos = mass.GetPos()
    print(f"Time: {system.GetChTime():.2f}s, Mass position: {mass_pos.y:.3f}m")