import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m


# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create systems

    #  Create the FEDA vehicle, set parameters, and initialize
    feda = veh.FEDA()
    feda.SetContactMethod(chrono.ChContactMethod_NSC)
    feda.SetChassisFixed(False)
    feda.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    feda.SetEngineType(veh.EngineModelType_SIMPLE)
    feda.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    feda.SetTireType(veh.TireModelType_TMEASY)
    feda.SetTireStepSize(1e-3)
    feda.Initialize()

    feda.SetChassisVisualizationType(veh.VisualizationType_NONE)
    feda.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    feda.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
    feda.SetTireVisualizationType(veh.VisualizationType_MESH)

    feda.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain

    terrain = veh.RigidTerrain(feda.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('FED-Alpha')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.2), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(feda.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # Simulation loop
    feda.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = feda.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        feda.Advance(step_size)
        vis.Advance(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step size
step_size = 1e-3

main()