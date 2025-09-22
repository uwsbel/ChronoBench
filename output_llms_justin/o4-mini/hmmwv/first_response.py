import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens  # only needed if you add sensors later
import os

# ------------------------------------------------------------------
# 1) Initialize Chrono and set the data path
# ------------------------------------------------------------------
chrono.SetChronoDataPath(os.path.join(os.getenv('HOME'), 'chrono_data'))
# Choose contact method
contact_method = chrono.ChContactMethod_SMC

# ------------------------------------------------------------------
# 2) Create and initialize the HMMWV vehicle
# ------------------------------------------------------------------
# Vehicle parameters
init_loc = chrono.ChVectorD(0, 0.6, 0)              # x, y, z
init_rot = chrono.Q_from_AngY(0)                    # yaw = 0
tire_model = veh.VehicleTireModelType_TMEASY
step_size = 1e-3                                   # integration step size

# Instantiate the vehicle
vehicle = veh.HMMWV_Vehicle(contact_method)
vehicle.SetInitPosition(chrono.ChCoordsys(init_loc, init_rot))
vehicle.SetTireType(tire_model)
vehicle.SetChassisFixed(False)
vehicle.Initialize()

# Visualization: primitive shapes for chassis, suspension, wheels, steering, tires
vis = veh.VisualizationType_PRIMITIVES
vehicle.SetChassisVisualizationType(vis)
vehicle.SetSuspensionVisualizationType(vis)
vehicle.SetSteeringVisualizationType(vis)
vehicle.SetWheelVisualizationType(vis)
vehicle.SetTireVisualizationType(vis)

# ------------------------------------------------------------------
# 3) Create the flat rigid terrain
# ------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())

# add a single patch of 20×20 m, thickness 0.1 m
patch = terrain.AddPatch(
    chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    chrono.ChVectorD(20, 20, 0.1))
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
# apply a tileable asphalt texture
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

terrain.Initialize()

# ------------------------------------------------------------------
# 4) Create the Irrlicht application for visualization
# ------------------------------------------------------------------
app = veh.ChVehicleIrrApp(vehicle,                # vehicle
                          "HMMWV on Flat Terrain",# window title
                          irr.dimension2du(1280, 720),
                          veh.VisualizationType_PRIMITIVES)
app.SetSkyBox()               # add a sky
app.AddTypicalLights()        # add .irr lights
app.SetChaseCamera(           # follow the vehicle
    chrono.ChVectorD(-8, 3, 0),
    5.0, 0.5)
app.SetTimestep(step_size)
app.AssetBindAll()
app.AssetUpdateAll()

# ------------------------------------------------------------------
# 5) Create the interactive driver (WASD + mouse)
# ------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
driver.Initialize()

# ------------------------------------------------------------------
# 6) Simulation loop
# ------------------------------------------------------------------
# run in real time at 50 fps
render_fps = 50
render_step = 1.0 / render_fps
next_frame_time = 0.0

system = vehicle.GetSystem()
time = 0.0

print("Starting simulation. Press WASD keys to drive, mouse to steer.")
while app.GetDevice().run():
    # current chrono time
    time = system.GetChTime()

    # driver inputs
    driver_inputs = driver.GetInputs()
    # Synchronize modules at current time
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(time, driver_inputs)

    # Render scene only at the desired frame rate
    if time >= next_frame_time:
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        next_frame_time += render_step

    # Advance the simulation of all modules by step_size
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

print("Simulation finished.")