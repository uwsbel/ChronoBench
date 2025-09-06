import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

"""
Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(6, -70, 0.5)
initRot = chrono.Q_from_AngAxis(1.57, chrono.ChVectorD(0, 0, 1))

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType.MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType.NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType.TMEASY

# Rigid terrain parameters
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Camera tracking point
trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)

# Contact method
contact_method = chrono.ChContactMethod.NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between render frames
render_step_size = 1.0 / 50  # 50 FPS

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Set collision system type
vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create terrain
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle.GetSystem())

# First terrain patch (original)
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        veh.GetDataFile("terrain/meshes/Highway_col.obj"),
                        True,  # invert normals
                        0.01,  # scale x
                        0.01,  # scale y
                        True)  # is mesh

# Visual shape for original terrain
vis_mesh = chrono.ChTriangleMeshConnected()
vis_mesh.LoadWavefront(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape.SetMesh(vis_mesh)
tri_mesh_shape.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/asphalt.jpg"))
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)

# New terrain patch with bump.obj
new_transform = chrono.ChCoordsysD(chrono.ChVectorD(0, -42, 0), chrono.QUNIT)
new_patch = terrain.AddPatch(patch_mat,
                            new_transform,
                            veh.GetDataFile("terrain/meshes/bump.obj"),
                            True,  # invert normals
                            1.0,   # scale x
                            1.0,   # scale y
                            True)  # is mesh

# Visual shape for new terrain patch
vis_mesh_bump = chrono.ChTriangleMeshConnected()
vis_mesh_bump.LoadWavefront(veh.GetDataFile("terrain/meshes/bump_vis.obj"), True, True)
tri_mesh_shape_bump = chrono.ChVisualShapeTriangleMesh()
tri_mesh_shape_bump.SetMesh(vis_mesh_bump)
tri_mesh_shape_bump.SetTexture(chrono.GetChronoDataFile("vehicle/terrain/textures/dirt.jpg"))
tri_mesh_shape_bump.SetTextureScale(6.0, 6.0)
tri_mesh_shape_bump.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
new_patch.GetGroundBody().AddVisualShape(tri_mesh_shape_bump)

terrain.Initialize()

# Create visualization
vis = irr.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)  # Fixed: Removed .GetVehicle()

# Create driver
driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % int(render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    driver_inputs = driver.GetInputs()
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)