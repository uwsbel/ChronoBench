import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc_truck = chrono.ChVector3d(0, -40, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(0, -15, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITITES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model
tire_model = veh.TireModelType_RIGID

# Rigid terrain
# terrain_model = veh.RigidTerrain.BOX
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Poon chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# # -----------------------------------------------------
# Load the truck vehicle, set parameters, and initialize
# # -----------------------------------------------------

truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.Initialize()

truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)

truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile('terrain/meshes/Highway_col.obj'),
    True, 0.01, False)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
tri_mesh = chrono.ChVisualShapeTriangleMesh()
tri_mesh.SetMesh(vis_mesh)
tri_mesh.SetMutable(False)
patch.GetGroundBody().AddVisualShape(tri_mesh)
terrain.Initialize()


# # ----------------------------------------
# # Create the sedan vehicle
# # ----------------------------------------
sedan = veh.BMW_E90(truck.GetSystem())
sedan.SetContactMethod(contact_method)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.SetTireType(tire_model)
sedan.SetTireStepSize(tire_step_size)
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)


# # -----------------------------------------------------
# Create the vehicle Irrlicht interface
# # -----------------------------------------------------

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Truck vehicle simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())


# # -----------------------------------------------------
# Create the driver system
# # -----------------------------------------------------

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 2.0)
driver.SetThrottleDelta(render_step_size)
driver.SetBrakingDelta(render_step_size / 10.0)
driver.Initialize()

# for power supply component list (NSC method only)
proc_powertrain_id = truck.GetTractor().GetPowertrain().GetTemplateName()
proc_powertrain = veh.ChPowertrain(stpk_truck, proc_powertrain_id, truck.GetTractor)
proc_powertrain.AddComponent(suppl_comp)
truck.GetTractor().GetPowertrain().SetProcPowertrain(proc_powertrain)


# # ------------------------------------
# Simulation loop
# # Save the state history
truck_state = []
sedan_state = []
state_history = chrono.ChTime()
while vis.Run() :
    time = truck.GetSystem().GetChTime()

    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # get driver inputs
    driver_inputs = driver.GetInputs()

    # update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs, terrain)
    sedan.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    # collect data from sensors (if added)
    truck_state.append(truck.GetState())
    sedan_state.append(sedan.GetState())
    # increment frame number
    step_number += 1