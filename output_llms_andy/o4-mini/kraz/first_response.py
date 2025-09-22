import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# --------------------------------------------------------------------------
# 1. General Chrono setup
# --------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())  
# (If you have a custom Chrono data folder, set it here)

# --------------------------------------------------------------------------
# 2. Create the vehicle
# --------------------------------------------------------------------------
# Choose Chrono::Vehicle Kraz
vehicle = veh.Kraz()
# Set initial position (x,y,z) and heading (about Z axis)
init_loc = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireStepSize(1e-3)
# Initialize the vehicle
vehicle.Initialize()

# Optional: choose visualization type for chassis, wheels, etc.
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)

# --------------------------------------------------------------------------
# 3. Create a rigid terrain
# --------------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())

# Define a material with specified friction and restitution
mat = chrono.ChMaterialSurfaceSMC()
mat.SetFriction(0.9)
mat.SetRestitution(0.1)
# Add a ground patch, size +/-50m in X,Y
ground = terrain.AddGround(mat)
ground.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

terrain.Initialize()

# --------------------------------------------------------------------------
# 4. Create the driver (Irrlicht GUI driver)
# --------------------------------------------------------------------------
app = irr.ChVisualSystemIrrlicht()
app.AttachVehicle(vehicle.GetVehicle())
app.SetWindowTitle("Kraz Vehicle on Rigid Terrain")
app.SetWindowSize(1280, 720)
app.Initialize()
app.AddTypicalLights()
# Position camera 8 m behind and 2 m above chassis, look at chassis center:
cam_pos = chrono.ChVectorD(-8, 0, 2)
cam_target = chrono.ChVectorD(0, 0, 0.5)
app.AddCamera(irr.ChCamera(irr.ChVectorD(0, -8, 2), irr.ChVectorD(0, 0, 0.5), 50))

# Use the built‐in IrrGui driver
driver = veh.ChIrrGuiDriver(app)
driver.SetInputDelay(0.2)  # optional user input delay

# --------------------------------------------------------------------------
# 5. Simulation parameters
# --------------------------------------------------------------------------
t_end = 20.0           # end time
step_size = 1e-3       # integration step size
render_step = 1.0 / 60 # render at 60 Hz
time = 0.0
next_render_time = 0.0

# --------------------------------------------------------------------------
# 6. Main simulation loop
# --------------------------------------------------------------------------
while app.Run() and time < t_end:
    # 6.1 Render scene at render_step intervals
    if time >= next_render_time:
        app.BeginScene()
        app.Render()
        app.EndScene()
        next_render_time += render_step

    # 6.2 Get driver inputs (steer, throttle, braking)
    driver_inputs = driver.GetInputs()

    # 6.3 Synchronize modules to the current time
    vehicle.Synchronize(time,
                        driver_inputs.m_steering,
                        driver_inputs.m_throttle,
                        driver_inputs.m_braking)
    terrain.Synchronize(time)
    driver.Synchronize(time)
    app.Synchronize("Kraz Vehicle", vehicle.GetChassisBody().GetPos(), 
                    vehicle.GetChassisBody().GetRot())

    # 6.4 Advance all modules by one step
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    driver.Advance(step_size)
    app.Advance(step_size)

    # 6.5 Increment time
    time += step_size

print("Simulation complete.")