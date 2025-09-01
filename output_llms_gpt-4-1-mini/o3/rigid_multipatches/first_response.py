#!/usr/bin/env python3
# =============================================================================
# HMMWV on multi-patch rigid terrain – PyChrono example
#
# Demonstrates:
#   • Full HMMWV (4-wheel-drive) vehicle
#   • RigidTerrain made of several independent patches
#       – two flat patches with different textures
#       – one bump created from a triangular mesh
#       – one height-map patch for rolling hills
#   • Irrlicht visualisation with chase-camera
#   • Interactive driver (keyboard) for steering / throttle / braking
#   • Real-time step synchronisation
# =============================================================================

import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh

# -----------------------------------------------------------------------------
# 1. Chrono initialisation
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # core data
veh.SetDataPath(veh.GetDataPath())                            # vehicle data
chrono.SetChronoMallocVerbose(False)

# Global simulation parameters
step_size     = 2e-3        # integration step (s)
tire_step     = step_size   # same for tire dynamics
render_step   = 1.0 / 60    # draw every 1/60s
sim_time      = 0.0

# -----------------------------------------------------------------------------
# 2. Create the physical system (NSC contact for rigid‐rigid)
# -----------------------------------------------------------------------------
system = chrono.ChSystemNSC()

# -----------------------------------------------------------------------------
# 3. Irrlicht application wrapper
# -----------------------------------------------------------------------------
app = veh.ChVehicleIrrApp(system, "HMMWV – multi-patch terrain", irr.dimension2du(1280, 720))
app.AddTypicalLights()
app.SetSkyBox()           # pretty background
app.SetShadows()
app.SetTimestep(step_size)
# (camera target will be set after vehicle creation)

# -----------------------------------------------------------------------------
# 4. Build the vehicle
# -----------------------------------------------------------------------------
init_pos   = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT)  # 1 m above ground
init_fwd   = chrono.ChVectorD(1, 0, 0)

fourWD     = True
engine     = veh.PowertrainModelType_SIMPLE
drivetrain = veh.DrivelineType_AWD

vehicle = veh.HMMWV_Full(system,
                         fixed=False,
                         driveType=drivetrain,
                         brakeType=veh.BrakeType_SIMPLE,
                         steeringType=veh.SteeringType_PITMAN_ARM,
                         contactMethod=chrono.ChContactMethod.NSC)
vehicle.Initialize(init_pos, init_fwd)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# tie the chase camera to the chassis
app.SetChaseCamera(vehicle.GetChassisBody(), 6.0, 0.5, 0.0)

# -----------------------------------------------------------------------------
# 5. Construct the multiterrain
# -----------------------------------------------------------------------------
terrain = veh.RigidTerrain(system)

# (a) concrete rectangle
patch_conc = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                              chrono.ChVectorD(200, 5, 0.2))
patch_conc.SetContactFrictionCoefficient(0.9)
patch_conc.SetMaterialSurface(chrono.ChVehicleUtils.CreateBoxMaterial(chrono.ChContactMethod.NSC, 0.9, 0.1))
patch_conc.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete.jpg"), 200, 5)

# (b) grass rectangle (beside concrete)
patch_grass = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 6, 0), chrono.QUNIT),
                               chrono.ChVectorD(200, 5, 0.2))
patch_grass.SetContactFrictionCoefficient(0.8)
patch_grass.SetElevation(0.0)
patch_grass.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 5)

# (c) bump using triangular mesh (OBJ)
bump_mesh = chrono.ChTriangleMeshConnected()
bump_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/bump_mesh.obj"), False, True)
bump_mesh.Transform(chrono.ChVectorF(10, -5, 0), chrono.ChMatrix33F(1))
patch_bump = terrain.AddPatch(bump_mesh, chrono.ChVectorD(10, -5, 0), chrono.ChQuaternionD(1, 0, 0, 0))
patch_bump.SetContactFrictionCoefficient(0.6)
patch_bump.SetTexture(chrono.GetChronoDataFile("terrain/textures/concrete_bump.jpg"), 1.0, 1.0)

# (d) height-map patch for rolling hills
hmap_file   = chrono.GetChronoDataFile("terrain/heightmaps/test64.bmp")  # 64x64 pixel grayscale
hmap_scale  = chrono.ChVectorD(1.0, 1.0, 0.5)    # x,y length per pixel, z scale
size_x      = 64 * hmap_scale.x
size_y      = 64 * hmap_scale.y
base        = chrono.ChCoordsysD(chrono.ChVectorD(-size_x*0.5, -20, 0), chrono.QUNIT)
patch_hmap  = terrain.AddPatch(base, hmap_file, hmap_scale, size_x, size_y, 0.0)
patch_hmap.SetContactFrictionCoefficient(0.75)
patch_hmap.SetTexture(chrono.GetChronoDataFile("terrain/textures/dirt.jpg"), size_x, size_y)

terrain.Initialize()

app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 6. Interactive driver
# -----------------------------------------------------------------------------
driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# -----------------------------------------------------------------------------
# 7. Realtime step timer
# -----------------------------------------------------------------------------
realtime_timer = chrono.ChRealtimeStepTimer()
render_accum   = 0.0

# -----------------------------------------------------------------------------
# 8. Main simulation loop
# -----------------------------------------------------------------------------
print("Controls:")
print("  W/S : throttle")
print("  A/D : steer")
print("  SPACE: brake")
print("  P   : toggle powertrain info")

while app.GetDevice().run():

    # Render if enough sim time has passed
    if (sim_time - render_accum) >= render_step:
        app.BeginScene()
        app.DrawAll()
        app.EndScene()
        render_accum = sim_time

    # Collect driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    vehicle.Synchronize(sim_time,
                        driver_inputs,
                        terrain)
    app.Synchronize("HMMWV demo", driver_inputs)

    # Advance state
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    app.Advance(step_size)

    # Move time forward, keep real-time
    sim_time += step_size
    realtime_timer.Spin(step_size)