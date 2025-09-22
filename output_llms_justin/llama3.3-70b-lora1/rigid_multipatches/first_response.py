import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m
import errno
import os
import math


# =============================================================================

def main():
    print("Copyright (c) 2017 projectchrono.org\n")

    # Create systems

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(contact_method)
    hmmwv.SetChassisCollisionType(chassis_collision_type)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    hmmwv.SetEngineType(engine_model)
    hmmwv.SetTransmissionType(transmission_model)
    hmmwv.SetDriveType(drive_type)
    hmmwv.SetSteeringType(steering_type)
    hmmwv.SetTireType(tire_model)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(chassis_vis_type)
    hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    hmmwv.SetSteeringVisualizationType(steering_vis_type)
    hmmwv.SetWheelVisualizationType(wheel_vis_type)
    hmmwv.SetTireVisualizationType(tire_vis_type)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain

    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patchmat = chrono.ChContactMaterialNSC()
    patchmat.SetFriction(0.9)
    patchmat.SetRestitution(0.01)
    if (contact_method == chrono.ChContactMethod_NSC):
        patchmat.SetYoungModulus(1e7)
    patch = terrain.AddPatch(patchmat, 
                             chrono.ChCoordsysd(chrono.ChVector3d(-10, 0, 0), chrono.QUNIT))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetLength(30.0)
    patch.SetWidth(30.0)

    patch = terrain.AddPatch(patchmat, 
                             chrono.ChCoordsysd(chrono.ChVector3d(60, 0, 0), chrono.QUNIT))
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetLength(40.0)
    patch.SetWidth(10.0)

    patch = terrain.AddPatch(patchmat, chrono.ChCoordsysd(chrono.ChVector3d(28, 0, 14), chrono.QUNIT), texture)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetLength(10.0)
    patch.SetWidth(28.0)
    patch.SetHeight(0.2)
    patch.SetZOffset(-0.1)

    patch = terrain.AddPatch(patchmat, 
                             chrono.ChCoordsysd(chrono.ChVector3d(90, 0.64, -30), chrono.QUNIT), 
                             hm_map)
    patch.SetColor(chrono.ChColor(0.2, 0.8, 0.2))
    patch.SetLength(100.0)
    patch.SetWidth(100.0)
    patch.SetHeight(15.0)
    patch.SetPlanes(80, 80)
    patch.SetHeightScale(0.3)

    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Rigid Terrain')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # Simulation loop

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of powertrain models (SHAFTS, SIMPLE)
engine_model = veh.EngineModelType_SHAFTS
transmission_model = veh.TransmissionModelType_AUTOMATIC_SHAFTS

# Drive type (FWD, RWD, or AWD)
drive_type = veh.DrivelineTypeWV_AWD

# Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0.0

# Point on plane for mesh creation
plane = chrono.ChPlane(chrono.ChVector3d(0, 0, terrainHeight), chrono.ChVector3d(0, 0, 1))
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
if (contact_method == chrono.ChContactMethod_NSC):
    patch_mat.SetYoungModulus(1e7)
texture = veh.RigidTerrainTexture(200, 200)
patch_mesh = chrono.ChTriangleMeshConnected()
patch_mesh.AddLayer(200, 200, plane, patch_mat, True)
patch_section = veh.RigidTerrainSection(
    patch_mesh, texture, 200, 200)
patch_section.SetTexture(veh.GetDataFile("terrain/textures/tile5.jpg"), 200, 200)

# Heightmap terrain
hmjp = veh.HeightMapJaggedProfile(veh.GetDataFile("terrain-heightmaps/bump.txt"))
hmjp.SetScale(2, 5, 10)
hm_mat = chrono.ChContactMaterialNSC()
hm_mat.SetFriction(0.9)
hm_mat.SetRestitution(0.01)
if (contact_method == chrono.ChContactMethod_NSC):
    hm_mat.SetYoungModulus(1e7)
hm_texture = veh.HeightMapTexture(hmjp.GetWidth(), hmjp.GetDepth())
hm = veh.HeightMap(hmjp, hm_texture, hm_mat)

# Simulation step sizes
step_size = 3e-3
tire_step_size = 1e-3

# Simulation loop render step size
render_step_size = 1.0 / 50  # FPS = 50

# Time response for steering and throttle
steering_time = 1.0
throttle_time = 1.0

# Output directory
out_dir = "-hmmwv"

# =============================================================================


result = main()
if result == 0 :
    print(&#x27;\nNEWTON_EULER demo completed successfully!&#x27; + chrono.GetChronoVersion())
else :
    print(&#x27;\nERROR: NEWTON_EULER demo failed!&#x27; + chrono.GetChronoVersion())