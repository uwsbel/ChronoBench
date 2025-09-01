import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# ------------------
# Parameters
# ------------------
timestep = 0.01  # Simulation timestep
terrain_length = 100  # Terrain length
terrain_width = 100  # Terrain width
vehicle_init_pos = chrono.ChVectorD(0, 1.0, 0)  # Initial vehicle position
vehicle_init_vel = chrono.ChVectorD(0, 0, 0)  # Initial vehicle velocity

# ------------------
# Create Chrono system
# ------------------
system = chrono.ChSystem_NSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ------------------
# Create terrain
# ------------------
terrain = chrono.ChTerrain(system)
terrain.Initialize(terrain_length, terrain_width, 0.05)
terrain.SetFriction(0.8)
terrain.SetRestitution(0.1)

# ------------------
# Create Kraz vehicle
# ------------------
vehicle = veh.ChWheeledVehicle(system, "Kraz/kraz_vehicle.json")
vehicle.Initialize(chrono.ChCoordsysD(vehicle_init_pos, chrono.QUNIT))
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# ------------------
# Create driver system
# ------------------
driver = veh.ChPathFollowerDriver(vehicle)
driver.Initialize()

# ------------------
# Create Irrlicht visualization
# ------------------
application = chronoirr.ChIrrApp(system)
application.SetSkyBox()
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 5, -10), vehicle_init_pos)

# ------------------
# Simulation loop
# ------------------
while application.GetDevice().run():
    # Synchronize vehicle and driver
    driver.Synchronize(timestep)

    # Advance simulation by one timestep
    system.DoStepDynamics(timestep)

    # Update visualization
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    application.EndScene()

    # Check for exit key (Esc)
    if application.GetDevice().getKeyState(chronoirr.KEY_ESCAPE):
        break