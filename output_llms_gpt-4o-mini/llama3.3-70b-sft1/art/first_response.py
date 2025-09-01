import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m


# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create systems

    #  Create the ARTcar vehicle, set parameters, and initialize
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisFixed(False)
    car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    car.SetEngineType(veh.EngineModelType_SIMPLE)
    car.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    car.SetTireType(veh.TireModelType_RIGID)
    car.Initialize()

    # Create the terrain

    terrain = veh.RigidTerrain(car.GetSystem())
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 200)
    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('dart')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.2), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(car.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.06)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Simulation loop

    # Number of simulation steps between miscellaneous events
    render_steps = m.ceil(render_step_size / step_size)

    # Initialize simulation frame counter and simulation time
    step_number = 0

    car.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = car.GetSystem().GetChTime()

        # Render scene and output POV-Ray data
        if (step_number % render_steps == 0) :
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            #vis.SavePOV()

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

        # Increment frame number
        step_number += 1

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step sizes
step_size = 2e-3;
render_step_size = 1.0 / 50  # FPS = 50

main()