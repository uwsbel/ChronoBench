import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# ---------------------------------------------------------------------
#
#  Demo code about using the Kraz vehicle
#
# ---------------------------------------------------------------------

# Create a Chrono system
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Set the collision margin
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)

# Create the terrain
terrain = veh.RigidTerrain(sys)
terrain.SetMetaInformation("terrain_friction", 0.9)
terrain.SetMetaInformation("terrain_restitution", 0.1)

# Create the vehicle
kraz = veh.Kraz(sys)

# Set initial vehicle position
init_pos = chrono.ChVector3d(0, 0, 0)
kraz.SetInitPosition(chrono.ChCoordsysd(init_pos, chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))))

# Set the vehicle contact method
kraz.SetContactMethod(veh.ChContactMethod.SMC)

# Initialize the vehicle
kraz.Initialize()

# Create a driver system
driver = veh.KrazDriver(kraz)
driver.SetSteering(0.0)
driver.SetThrottle(0.0)
driver.SetBraking(0.0)

# ---------------------------------------------------------------------
#
#  Create the Irrlicht application to visualize the system
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Kraz Vehicle Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(5, 5, -5))
vis.AddTypicalLights()
vis.AddSkyBox()

# ---------------------------------------------------------------------
#
#  Simulation loop
#

time_step = 0.01
time_total = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Update the driver
    driver.Synchronize(time_step)

    # Advance the simulation
    sys.DoStepDynamics(time_step)

    time_total += time_step

    # Print simulation time
    # print(f"Time: {time_total}")