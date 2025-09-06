import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. System Initialization
system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVector3d(0, 0, 0))  # Disable gravity for clear demonstration

# 2. Create Ground Body
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground_box = chrono.ChVisualShapeBox(0.4, 0.2, 0.2)
ground_box.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
ground.AddVisualShape(ground_box)
system.Add(ground)

# 3. Create Mass Body
mass_body = chrono.ChBody()
mass_body.SetMass(10)  # 10 kg mass
mass_body.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
mass_body.SetPos(chrono.ChVector3d(2, 0, 0))  # Initial offset
sphere = chrono.ChVisualShapeSphere(0.3)
sphere.SetColor(chrono.ChColor(0.2, 0.6, 0.9))
mass_body.AddVisualShape(sphere)
system.Add(mass_body)

# 4. Create Spring-Damper (TSDA Link)
spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass_body, True, 
                  chrono.ChVector3d(0, 0, 0),  # Point on ground
                  chrono.ChVector3d(0, 0, 0))   # Point on mass

# Set spring parameters
spring.SetSpringCoefficient(100)   # 100 N/m stiffness
spring.SetDampingCoefficient(5)    # 5 N·s/m damping

# Add spring visualization
spring_vis = chrono.ChVisualShapeSpring()
spring_vis.SetNumCoils(20)
spring_vis.SetSpringRadius(0.1)
spring_vis.SetDiameter(0.05)
spring_vis.SetColor(chrono.ChColor(0.2, 0.8, 0.3))
spring.AddVisualShape(spring_vis)

system.Add(spring)

# 5. Initialize Visualization System
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Mass-Spring-Damper System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 1, 3))
vis.AddTypicalLights()

# 6. Simulation Loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)