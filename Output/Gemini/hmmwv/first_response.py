import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# ---------------------
# Simulation parameters
# ---------------------
time_step = 1e-3
fps = 50
terrain_length = 100.0  # in meters
terrain_width = 100.0  # in meters

# ------------------
#  Create the system
# ------------------
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------
# Create the HMMWV
# ---------------
# Use the provided JSON file for HMMWV parameters (replace with your actual path)
hmmwv_json = "hmmwv/vehicle/HMMWV_Vehicle.json"
hmmwv = veh.HMMWV_Full(hmmwv_json)
hmmwv.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0)))
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(0, 1, 0))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)
hmmwv.SetVehicleStepSize(time_step)
system.Add(hmmwv.GetVehicle())

# -------------------
# Create the terrain
# -------------------
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.Initialize(patch_mat, terrain_length, terrain_width, 0.8, 0.04)
terrain.AddVisualizationAssets(chrono.VisualizationType_TEXTURE)
terrain_asset = terrain.GetGroundBody().GetAssets()[0]
terrain_asset.SetTextureFilename(chrono.GetChronoDataFile("textures/dirt.jpg"))
system.Add(terrain.GetGroundBody())

# -------------------
# Create the driver
# -------------------
driver = veh.ChInteractiveDriverIRR(system)
driver.Initialize()

# ---------------
# Irrlicht Setup
# ---------------
vis = chronoirr.ChIrrApp(system, "HMMWV Simulation", chronoirr.dimension2du(1024, 768))
vis.AddTypicalSky()
vis.AddTypicalLights()
vis.AddTypicalCamera(chronoirr.vector3df(30, 30, 30), chronoirr.vector3df(0, 1.5, 0))
vis.SetSymbolscale(0.01)
vis.AssetBindAll()
vis.AssetUpdateAll()

# ----------------
# Simulation loop
# ----------------
while vis.Run():
    time = system.GetChTime()

    # Driver inputs
    driver_inputs = driver.GetInputs()

    # Update the vehicle
    hmmwv.Synchronize(time, driver_inputs, terrain)

    # Advance simulation
    system.DoStepDynamics(time_step)

    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Enforce real-time simulation
    chrono.ChThSleepUntil(time + 1.0 / fps)