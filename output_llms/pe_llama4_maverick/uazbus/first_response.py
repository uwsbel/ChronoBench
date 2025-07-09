import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# ==============================================================================

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50

# ==============================================================================

# JSON file for vehicle model
vehicle_file = "uazbus/vehicle/UAZBUS.json"

# JSON files for terrain
rigidterrain_file = "terrain/RigidPlane.json"

# ==============================================================================

def main():
    # Create systems
    vehicle_sys = chrono.ChSystemSMC()

    # Create the vehicle
    my_vehicle = veh.UAZBUS(vehicle_sys, vehicle_file)
    my_vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    my_vehicle.Initialize()

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle_sys, rigidterrain_file)
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
    terrain.Initialize()

    # Create the driver system
    driver = veh.ChDriver(my_vehicle.GetVehicle())
    driver.Initialize()

    # Create the Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(vehicle_sys)
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('UAZBUS Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 4, -6))
    vis.AddTypicalLights()

    # Simulation loop
    render_step_size = 1.0 / 50
    render_time = 0

    while vis.Run():
        time = vehicle_sys.GetChTime()

        # Render scene
        if (time >= render_time):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_time += render_step_size

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        terrain.Synchronize(time)
        my_vehicle.Synchronize(time, driver_inputs, terrain)
        driver.Synchronize(time)

        # Advance simulation for one timestep for all modules
        terrain.Advance(step_size)
        my_vehicle.Advance(step_size)
        driver.Advance(step_size)

        # Advance state of entire system (containing both the terrain and vehicle)
        vehicle_sys.DoStepDynamics(step_size)

    return 0

if __name__ == "__main__":
    main()