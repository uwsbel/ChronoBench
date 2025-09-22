import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the physical system
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity vector

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ANCF Cable Beam Simulation')
vis.SetSymbolscale(0.01)
vis.SetShowInfos(True)
vis.SetCameraPosition(chrono.ChVectorD(0, 5, 10))
vis.SetCameraRotation(chrono.Q_from_AngAxis(chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 0, 1)))

my_system.SetVisualSystem(vis)
application = chronoirr.ChIrrApp(my_system, vis)
application.AddAllTypicalLights()
application.AddAllTypicalCameraPositions()

# Create the ANCF cable beam
num_elements = 20
length = 10.0
radius = 0.1
young_mod = 2e11  # Young's modulus for steel
poisson = 0.3
density = 7850  # Density for steel

beam = chrono.ChBodyEasyCableANCF(
    my_system,
    num_elements,
    length,
    radius,
    young_mod,
    poisson,
    density
)
beam.SetPos(chrono.ChVectorD(0, 0, 0))  # Position at origin, aligned along X-axis
my_system.Add(beam)

# Constrain the first node to the ground (hinge)
link = chrono.ChLinkLockPointFrame()
link.Initialize(beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
my_system.Add(link)

# Initialize visualization
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStepIfNoErrors(0.001)  # Time step of 1ms
    application.EndScene()