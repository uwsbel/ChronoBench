import pychrono as chrono
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Create the physical system
system = chrono.ChSystemNSC()

# 2. Add the required physical systems and objects

# --- Ground Body (fixed) ---
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# --- Mass Body (movable) ---
mass = chrono.ChBodyEasyBox(0.2, 0.2, 0.2,  # size x, y, z
                            1.0,            # density
                            True,           # visualization?
                            True)           # collision?
mass.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(mass)

# --- Spring-Damper (ChLinkTSDA) ---
# Anchor point on ground and mass
pos_ground = chrono.ChVectorD(0, 0, 0)
pos_mass = chrono.ChVectorD(0, -0.1, 0)  # attach to bottom of box

spring = chrono.ChLinkTSDA()
spring.Initialize(ground, mass, False, pos_ground, mass.GetPos() + pos_mass)

# Default parameters: K=100 N/m, R=5 N*s/m, rest length = distance between points
spring.SetSpringCoefficient(100)
spring.SetDampingCoefficient(5)
spring.SetRestLength((pos_ground - (mass.GetPos() + pos_mass)).Length())

system.AddLink(spring)

# 3. Set necessary default parameters

# Give the mass an initial velocity for demonstration
mass.SetPos_dt(chrono.ChVectorD(0, 0, 0))

# --- Visualization ---

# Create Irrlicht application
app = chronoirr.ChVisualSystemIrrlicht()
app.AttachSystem(system)
app.SetWindowSize(1024, 768)
app.SetWindowTitle("PyChrono: Mass-Spring-Damper (ChLinkTSDA)")
app.Initialize()
app.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.AddSkyBox()
app.AddCamera(chrono.ChVectorD(1.0, 0.7, 1.0), chrono.ChVectorD(0, 0.3, 0))
app.AddTypicalLights()

# Visualize the spring as a cylinder
spring_shape = chrono.ChLinkTSDAVisualShape()
spring_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.8))
spring_shape.SetResolution(80)
spring_shape.SetRadius(0.03)
spring.AddVisualShape(spring_shape)

# Visualize the mass as a colored box
mass.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))

# Simulation loop
app.AssetBindAll()
app.AssetUpdateAll()

timestep = 0.001

while app.Run():
    app.BeginScene()
    app.Render()
    app.EndScene()
    system.DoStepDynamics(timestep)