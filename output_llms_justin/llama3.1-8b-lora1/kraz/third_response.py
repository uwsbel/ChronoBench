import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# -----------------
# Create systems
# -----------------

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# ---------------
# Load truck HMMWV
# ---------------
truck = veh.HMMWV()
truck.SetContactMethod(chrono.ChContactMethod_NSC)
truck.SetChassisCollisionType(veh.CollisionType_NONE)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-50, 0, 0.4), chrono.QUNIT))
truck.Initialize()

truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ---------------
# Load sedan
# ---------------
sedan = veh.Sedan()
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisCollisionType(veh.CollisionType_NONE)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(50, 0, 0.4), chrono.QUNIT))
sedan.Initialize()

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
sedan.SetSteeringVisualizationType(veh.VisualizationType_MESH)
sedan.SetSuspensionVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
sedan.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# -------------------------------------
# Create the Irrlicht video system
# Create the driver system
# -------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Two Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.5), 8.0, 1.2)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())

driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver.SetSteeringDelta(1.0 / (render_step_size / steering_time))
driver.SetThrottleDelta(1.0 / (render_step_size / throttle_time))
driver.SetBrakingDelta(1.0 / (render_step_size / braking_time))

driver.Initialize()

# ------------------
# Create the terrain
# ------------------
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1), chrono.QUNIT), 
    100, 100)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# -----------------------
# Initialize output system
# -----------------------
out = veh.ChVehicleOutputASCII()
out.SetStepSize(step_size)
out.SetOutput(veh.TrailerOutput(veh.TrailerKey(truck.GetTractor(), truck.GetTrailer()), "Trailer", step_size, "\nTime: ", "\n  X Y Z: "))
out.Initialize()

# ---------------
# Initialize simulation
# ---------------
truck.GetSystem().SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

sedan.GetSystem().SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

# -----------------------
# Simulation loop
# -----------------------
time = 0
render_steps = math.ceil(render_step_size / step_size)

while vis.Run() :
    time = truck.GetSystem().GetChTime()

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    sedan.Synchronize(time, driver_inputs, terrain)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis.Advance(step_size)
    sedan.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)