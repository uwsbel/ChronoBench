import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False) 
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.QUNIT))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the SCM terrain
    terrain = veh.SCMterrain(hmmwv.GetSystem())
    patch_mat = terrain.AddPatch(terrain.Length, terrain.Width)
    patch_mat.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch_mat.SetCollisionType(chrono.ChCollisionType_NONE)
    patch_mat.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch_mat.SetSimulationType(veh.SCMterrain.PatchSimulationType_RIGID)
    patch_mat.SetRestractionType(veh.SCMterrain.PatchRestractionType_NONE)
    patch_mat.SetTractionType(veh.SCMterrain.PatchTractionType_NONE)
    patch_mat.SetSetpolarType(veh.SCMterrain.PatchSetpolarType_NONE)
    patch_mat.SetSetpolarParameters(0.0, 0.0)
    patch_mat.Initialize()

    # Create the Irrlicht visualization
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV on SCM terrain')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        # Draw scene
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

# Initial vehicle location
initLoc = chrono.ChVector3d(-10, -2, 0.6)

# Simulation step sizes
step_size = 2e-3
tire_step_size = step_size

# Simulation end time
t_end = 100

# SCM patch size
patch_size = 20

# SCM patch offset
patch_offset = 0

# SCM soil parameters
soil_model = veh.SCMterrain.SoilModel_NISU
soil_rigidity = 3e7
soil_damping = 0.8
soil_rest_height = 0.05
soil_falloff_distance = 1.0

# Visualization type for vehicle parts (PRIMITIVES, MESH)
chassis_vis_type = veh.VisualizationType_PRIMITIVES
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH

# =============================================================================

main()