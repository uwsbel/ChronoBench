import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os # For checking file existence and path joining

def main():
    # Simulation step sizes
    step_size = 2e-3
    # Tire step size_ tire_step_size was used in the original script,
    # it must be smaller than or equal to simulation step size.
    tire_step_size = 1e-3 

    # Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    
    # Instruction 1: Initial Position Update
    # Original: chrono.ChVector3d(-10, -2, 0.6)
    # New: chrono.ChVector3d(6, -70, 0.5)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(6, -70, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size) # tire_step_size defined above
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Set the collision system type for the entire system
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # Instruction 2 & 4: Terrain Definition Simplification and Remove Old Patches
    # All old patch definitions (patch1, patch2, patch3, patch4) are removed.

    # Define the contact material for the new single terrain patch
    # Friction 0.9, Restitution 0.01 (as per original patch1_mat and instructions)
    patch_material = chrono.ChContactMaterialNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)
    
    # Regarding "contact material thickness of 0.01":
    # For NSC material, this is interpreted as setting the collision margin for the 
    # terrain's ground body's collision model. This is important for mesh collisions.
    ground_body = terrain.GetGroundBody()
    ground_body.GetCollisionModel().SetDefaultSuggestedMargin(0.01)

    # Add the single new terrain patch using a collision mesh
    # Mesh file: vehicle/terrain/meshes/Highway_col.obj, accessed via veh.GetDataFile()
    # veh.GetDataFile() prepends the vehicle data path.
    collision_mesh_file = veh.GetDataFile("terrain/meshes/Highway_col.obj")
    if not os.path.exists(collision_mesh_file):
         print(f"FATAL ERROR: Collision mesh file not found: {collision_mesh_file}")
         print(f"Ensure CHRONO_DATA_DIR is set correctly and the file exists at the expected location: {os.path.abspath(collision_mesh_file)}")
         return 1 # Indicate an error

    # AddPatch for mesh collision. Position of the mesh is set here (e.g., origin).
    # This function call adds collision geometry to the ground_body.
    terrain.AddPatch(patch_material,
                     chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), # Position of the terrain mesh
                     collision_mesh_file)
    # Note: RigidTerrain::AddPatch for a mesh file typically only sets up collision, not visualization.

    # Instruction 3: Add Visual Mesh to Terrain
    # Instruction 5: Mesh-Related Functions (using Chrono's API for mesh visualization)
    # Visual mesh file: terrain/meshes/Highway_vis.obj
    visual_mesh_file = veh.GetDataFile("terrain/meshes/Highway_vis.obj")
    if not os.path.exists(visual_mesh_file):
        print(f"FATAL ERROR: Visual mesh file not found: {visual_mesh_file}")
        print(f"Ensure CHRONO_DATA_DIR is set correctly and the file exists at the expected location: {os.path.abspath(visual_mesh_file)}")
        return 1 # Indicate an error

    # These are the "mesh-related functions" for creating and adding visual shapes:
    # 1. Create a ChTriangleMeshConnected object
    vis_mesh_trimesh = chrono.ChTriangleMeshConnected()
    # 2. Load the mesh from an OBJ file
    vis_mesh_trimesh.LoadWavefrontMesh(visual_mesh_file, False, True) # (filename, load_normals, load_uv_coords for texture)

    # 3. Create a ChVisualShapeTriangleMesh
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    # 4. Assign the loaded mesh to the visual shape
    vis_shape.SetMesh(vis_mesh_trimesh)
    vis_shape.SetName("highway_visual_mesh") # Optional: for identification in viewers or logs
    # Optional: Apply a texture if the OBJ/MTL doesn't define one or to override.
    # Example: vis_shape.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"))
    
    # 5. Add this visual shape to the terrain's ground body
    ground_body.AddVisualShape(vis_shape)
    # The ground body is typically fixed, and its visualization is now handled by this shape.

    terrain.Initialize() # Finalize terrain setup, builds collision models.


    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Mesh Terrain Demo') # Updated title for clarity
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5) # Default chase camera settings
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # General Chrono data file
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Enable real-time simulation for the vehicle
    hmmwv.GetVehicle().EnableRealtime(True)

    # Simulation loop
    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        # Render scene
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

# Set the path to the vehicle JSON data files.
# This should be set before any vehicle data is loaded.
# chrono.GetChronoDataPath() returns the path to the Chrono data directory.
# os.path.join is used for platform-independent path construction.
veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle', ''))

# Main execution
if __name__ == '__main__':
    main()