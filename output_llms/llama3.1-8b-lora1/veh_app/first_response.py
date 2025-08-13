import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math as m

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the Gator vehicle, set parameters, and initialize
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisFixed(False) 
    gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -1100, 1.0), chrono.QUNIT))
    gator.SetEngineType(veh.EngineModelType_SIMPLE)
    gator.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    gator.SetDriveType(veh.DrivelineTypeWV_AWD)
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.Initialize()

    gator.SetChassisVisualizationType(veh.VisualizationType_NONE)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    gator.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the (spherical) terrain
    terrain = veh.RigidTerrain(gator.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.ChCoordsysd(chrono.VNULL, chrono.QUNIT), 
                             200.0, 100.0)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create the driver system
    driver = veh.ChDriverSQL(gator.GetVehicle())
    driver.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Gator')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, -0.8, 0.3), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(gator.GetVehicle())

    # Create the sensor manager for the visual system
    sens_manager = vis.GetSensorManager()
    sens_manager.addCamera(chrono.CameraParams(0.4, 1024, 768, 3, 1), 15)
    sens_manager.initialize()

    # Simulation loop
    gator.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = gator.GetSystem().GetChTime()

        # Driver inputs
        driver_inputs = driver.GetInputs()
        gator.GetVehicle().InputDriverSQL(driver_inputs, time)

        # Update vehicle
        gator.Update()
        terrain.Update()

        # Update sensor manager
        vis.BeginScene()
        sens_manager.BeginScene()
        sens_manager.Render()
        vis.EndScene()

        # Set state for the next timestep
        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time)
        vis.Synchronize(time)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.Advance(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(-10, -1100, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# =============================================================================

main()