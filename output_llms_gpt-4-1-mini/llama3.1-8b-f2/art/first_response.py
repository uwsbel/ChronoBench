import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the ARTcar vehicle, set parameters, and initialize
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisFixed(False);
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    car.SetEngineType(veh.EngineModelType_SIMPLE)
    car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    car.SetTireType(veh.TireModelType_TMEASY)
    car.SetTireStepSize(1e-3)
    car.Initialize()

    car.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    car.SetWheelVisualizationType(veh.VisualizationType_NONE)
    car.SetTireVisualizationType(veh.VisualizationType_MESH)

    car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('ARTcar')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(car.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)

    # Set the time response for steering and throttle keyboard inputs.
    steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0  # time to go from 0 to +1
    braking_time = 0.3   # time to go from 0 to +1
    driver.SetSteeringDelta(10 * step_size / steering_time)
    driver.SetThrottleDelta(10 * step_size / throttle_time)
    driver.SetBrakingDelta(10 * step_size / braking_time)

    driver.Initialize()

    # Simulation loop
    car.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = car.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        car.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        car.Advance(step_size)
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