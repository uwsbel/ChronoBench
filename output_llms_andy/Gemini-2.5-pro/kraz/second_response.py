import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import os

# --- Chrono data path setup ---
# This setup attempts to be robust. If CHRONO_DATA_DIR is not set, it tries a common relative path.
# You might need to adjust 'fallback_chrono_data_dir' if CHRONO_DATA_DIR is not available.
try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
except KeyError:
    print("Warning: CHRONO_DATA_DIR environment variable not set.")
    # Fallback strategy (adjust as needed for your environment)
    # This assumes the script is run from a directory where 'chrono_data' is a sibling or accessible via relative path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fallback_chrono_data_dir = os.path.join(script_dir, "..", "..", "chrono_data") # Common for SDK examples
    if not os.path.exists(fallback_chrono_data_dir):
        fallback_chrono_data_dir = os.path.join(script_dir, "chrono_data") # e.g. if chrono_data is in the same dir
    
    if os.path.exists(fallback_chrono_data_dir):
        print(f"Using fallback Chrono data path: {fallback_chrono_data_dir}")
        chrono.SetChronoDataPath(fallback_chrono_data_dir)
    else:
        print(f"Error: Chrono data directory not found at {fallback_chrono_data_dir} or via CHRONO_DATA_DIR.")
        print("Please ensure CHRONO_DATA_DIR is set or the fallback path is correct.")
        exit(1)

veh_data_path = os.path.join(chrono.GetChronoDataPath(), 'vehicle', '')
veh.SetDataPath(veh_data_path)


# --- Instruction: Changed initial vehicle location ---
# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-15, 0, 0.5)  # Changed from (0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # Assuming no change in rotation

# Visualization type for vehicle parts
vis_type = veh.Visualization.Type_MESH  # Corrected enum path

# Collision type for chassis
chassis_collision_type = veh.Collision.Type_NONE  # Corrected enum path

# Type of tire model
tire_model = veh.TireModel.Type_TMEASY  # Corrected enum path

# Rigid terrain parameters
terrainHeight = 0      # Reference height of the terrain plane
terrainLength = 200.0  # size in X direction (increased for longer maneuver)
terrainWidth = 200.0   # size in Y direction

# --- Instruction: Updated the track point for the camera ---
# Point on chassis tracked by the camera (relative to chassis origin)
trackPoint = chrono.ChVector3d(3.0, 0.0, 2.1)  # Changed from (0,0, 2.1)

# Contact method
contact_method = chrono.ChContactMethod_NSC
# contact_vis = False # This variable was defined but not used. Removed for clarity.

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size  # To be applied to the vehicle

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Create the Kraz vehicle
vehicle = veh.Kraz()
vehicle.SetContactMethod(contact_method)  # This sets up the ChSystem within the vehicle

# CRITICAL: Set collision system type for the ChSystem BEFORE Initializing bodies/terrain.
# This must be done after the system is associated with the vehicle (e.g., by SetContactMethod)
# and before Initialize() is called on vehicle or terrain.
if vehicle.GetSystem():
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
else:
    # This case should ideally not happen if Kraz() and SetContactMethod() work as expected.
    raise Exception("Vehicle system not found after Kraz() and SetContactMethod(). Cannot set collision system type.")

vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)  # CORRECTED: Apply the chosen tire model
vehicle.SetTireStepSize(tire_step_size)  # CORRECTED: Apply tire step size
vehicle.Initialize()

# Set visualization types for vehicle components (using corrected single-argument methods)
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
# Using PRIMITIVES for suspension as MESH can be heavy and may not always be available/configured
vehicle.SetSuspensionVisualizationType(veh.Visualization.Type_PRIMITIVES)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

# Create the terrain
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())  # Use the system from the vehicle
# Terrain patch defined at terrainHeight
patch_coordsys = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrainHeight), chrono.QUNIT)
patch = terrain.AddPatch(patch_mat, patch_coordsys, terrainLength, terrainWidth)

patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz Double Lane Change Demo')
vis.SetWindowSize(1280, 1024)
# --- Instruction: Adjusted the chase camera parameters ---
vis.SetChaseCamera(trackPoint, 25.0, 10.5)  # chase_dist=25.0, chase_height=10.5
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
# Attach the vehicle itself (GetVehicle() returns the ChWheeledVehicle)
vis.AttachVehicle(vehicle.GetVehicle())


# --- Instruction: Double lane change maneuver ---
# The ChInteractiveDriverIRR is replaced by scripted inputs.

# Parameters for the double lane change (DLC) maneuver
dlc_steer_amplitude = 0.28   # Max
print("error happened with only start ```python")