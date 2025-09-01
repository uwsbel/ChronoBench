import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    step_size = 0.005

    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    # Create the terrain
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # Create the vehicle
    vehicle = veh.CarVehicle(sys)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -1.5, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
    vehicle.Initialize()

    vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()

    vehicle.GetSystem().SetChTime(0)

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Differential Steering')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    # Visualization types for vehicle parts (PRIMITIVES, MESH, or NONE)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Output
    ofile = open('output/driving_data.txt', 'w')
    output = veh.ChVehicleOutput(veh.ChVehicleOutput.FORMAT_TAB_SEPARATED, ofile)
    output.AddOutput(veh.ChVehicleOutput.OutputType_DRIVING_DATA)
    output.Initialize()

    # Drive the vehicle
    steering_force = 0.0
    driving_style = veh.DrivingStyle_NONE
    vehicle.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        # End simulation
        if (time >= 3):
            break

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = vehicle.GetDriverInputs()

        # Update modules (process inputs from other modules)
        # Commented out because not needed in this example
        # terrain.Update()
        # vehicle.Update(time, driver_inputs, terrain)
        # vis.Update(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        # Commented out because not needed in this example
        # terrain.Advance(step_size)
        # vehicle.Advance(step_size)
        # vis.Advance(step_size)

        # Update output
        output.Update(time, vehicle)

        # Drive the vehicle
        if (driving_style == veh.DrivingStyle_KEYBOARD):
            steering_force = driver_inputs-steering_force
            steering = min(max(steering_force, -1.0), 1.0)
            vehicle.SynchronizeSteering(steering)
        elif (driving_style == veh.DrivingStyle_NONE):
            # Do nothing
            pass

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

main()