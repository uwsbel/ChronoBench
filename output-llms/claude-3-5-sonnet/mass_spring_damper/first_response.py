import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the Chrono system
chrono.SetChronoDataPath("../../../data/")
my_system = chrono.ChSystemNSC()

# Set the default gravity acceleration
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground
ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)

# Create the mass (represented by a sphere)
mass = chrono.ChBodyEasySphere(0.1, 1000)  # radius = 0.1, density = 1000
mass.SetPos(chrono.ChVectorD(0, 0.5, 0))  # Initial position
my_system.Add(mass)

# Create the spring-damper
spring_damper = chrono.ChLinkTSDA()
spring_damper.Initialize(ground, mass, True, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0.5, 0))
spring_damper.SetSpringCoefficient(50)  # Spring constant
spring_damper.SetDampingCoefficient(1)  # Damping coefficient
my_system.Add(spring_damper)

# Add visualization assets
col_g = chrono.ChColorAsset()
col_g.SetColor(chrono.ChColor(0.3, 0.3, 0.3))  # Gray color for ground
ground.AddAsset(col_g)

col_m = chrono.ChColorAsset()
col_m.SetColor(chrono.ChColor(0.6, 0.2, 0.2))  # Red color for mass
mass.AddAsset(col_m)

# Add a visualization asset for the spring
spring_asset = chrono.ChPointPointSpring(0.01, 80, 15)
spring_asset.SetColor(chrono.ChColor(0, 0.8, 0.8))
spring_damper.AddAsset(spring_asset)

# Create the Irrlicht visualization
application = chronoirr.ChIrrApp(my_system, "Mass-Spring-Damper Demo", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddTypicalCamera(chronoirr.vector3df(1, 0.5, -1))
application.AddTypicalLights()

# Run the simulation
application.AssetBindAll()
application.AssetUpdateAll()
application.SetTimestep(0.005)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()