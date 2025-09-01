import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ------------------------------------------------------------------------
# Setup Chrono paths and parameters
# ------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location/orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization & collision parameters
vis_type = veh.VisualizationType_MESH
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_TMEASY

contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0/50

# ------------------------------------------------------------------------
# Create and initialize the Gator vehicle
# ------------------------------------------------------------------------
vehicle = veh.Gator()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

# Visualization types
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Use the Bullet collision engine
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# ------------------------------------------------------------------------
# Create a multi‐patch rigid terrain
# ------------------------------------------------------------------------
terrain = veh.RigidTerrain(vehicle.GetSystem())
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.9)
mat.SetRestitution(0.01)

# Patch dimensions
patch_size_x = 50.0
patch_size_y = 50.0

# 1) SW Patch: tiling tile4.jpg
patch1_cs = chrono.ChCoordsysd(chrono.ChVector3d(-25, -25, 0.0), chrono.QUNIT)
patch1 = terrain.AddPatch(mat, patch1_cs, patch_size_x, patch_size_y)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"),  10, 10)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# 2) SE Patch: grass texture
patch2_cs = chrono.ChCoordsysd(chrono.ChVector3d(25, -25, 0.0), chrono.QUNIT)
patch2 = terrain.AddPatch(mat, patch2_cs, patch_size_x, patch_size_y)
patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 10, 10)
patch2.SetColor(chrono.ChColor(0.4, 0.8, 0.4))

# 3) NW Patch: concrete
patch3_cs = chrono.ChCoordsysd(chrono.ChVector3d(-25, 25, 0.0), chrono.QUNIT)
patch3 = terrain.AddPatch(mat, patch3_cs, patch_size_x, patch_size_y)
patch3.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 10, 10)
patch3.SetColor(chrono.ChColor(0.7, 0.7, 0.7))

# 4) NE Patch: use a height map to create undulations
#    We assume in ChronoData: vehicle/terrain/heightmaps/hilly.png
patch4_cs = chrono.ChCoordsysd(chrono.ChVector3d(25, 25, 0.0), chrono.QUNIT)
# The Python binding uses SetHeightMap(file, Lx, Ly, vertical_scale)
patch4 = terrain.AddPatch(mat, patch4_cs, patch_size_x, patch_size_y)
patch4.SetHeightMap(veh.GetDataFile("terrain/heightmaps/hilly.png"),
                    patch_size_x, patch_size_y, 5.0)
# You can still overlay a texture on top of the height map
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 10, 10)
patch4.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()

# Add a simple bump: a small fixed box on the SW patch
bump = chrono.ChBodyEasyBox(2.0, 2.0, 0.5,    # x,y,z dims
                            1000,              # density
                            True, True)        # visualization, collision
bump.SetPos(chrono.ChVectorD(-25, -25, 0.25))  # center of SW patch
bump.SetBodyFixed(True)
bump_material = bump.GetMaterialSurfaceNSC()
bump_material.SetFriction(mat.GetFriction())
bump_material.SetRestitution(mat.GetRestitution())
vehicle.GetSystem().Add(bump)

# ------------------------------------------------------------------------
# Create the Irrlicht visualization and driver
# ------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Gator on multi-patch terrain')
vis.SetWindowSize(1280, 1024)
# chase camera
vis.SetChaseCamera(chrono.ChVector3d(-3, 0, 1.1), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

driver = veh.ChInteractiveDriverIRR(vis)
# steering, throttle, braking time constants
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# ------------------------------------------------------------------------
# Simulation loop
# ------------------------------------------------------------------------
print("Vehicle mass: ", vehicle.GetVehicle().GetMass())

render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    t = vehicle.GetSystem().GetChTime()

    # render
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(t)
    terrain.Synchronize(t)
    vehicle.Synchronize(t, driver_inputs, terrain)
    vis.Synchronize(t, driver_inputs)

    # advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)