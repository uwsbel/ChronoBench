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
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
    vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # Output
    ofile = open('output/driving_data.txt', 'w')
    lstr = 't, chassis_speed_x, chassis_speed_y, chassis_speed_z, suspension_l_straight, suspension_l_bend, steering_l, steering_r\n'
    ofile.write(lstr)
    ofile.flush()

    # Driving
    st_time = 0
    en_time = 3
    steering_k = 2
    steering_t0 = 0
    steering_t1 = 1
    steering_cycle = 0

    vehicle_output = []

    # Simulation loop
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        # End simulation
        if (time >= en_time):
            break

        # Output vehicle data
        if (time > st_time):
            str_out = "%f, %f, %f, %f, %f, %f, %f, %f\n" % (time, 
                vehicle.GetChassis().GetLinearVelocity().x, 
                vehicle.GetChassis().GetLinearVelocity().y, 
                vehicle.GetChassis().GetLinearVelocity().z, 
                vehicle.GetSuspension(0).GetLength(), 
                vehicle.GetSuspension(1).GetLength(), 
                vehicle.GetSteering(0).GetAngle(), 
                vehicle.GetSteering(1).GetAngle())
            ofile.write(str_out)
            ofile.flush()

        # Irrlicht interface
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance simulation for one timestep for all parts (process inputs from other system components)
        vehicle.Synchronize(time)
        terrain.Synchronize(time)
        vis.Synchronize(time, step_size)

        # Advance state of entire system (process inputs from other system components)
        sys.DoStepDynamics(step_size)

        # Increment cycle number
        steering_cycle += 1

    return 0

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

main()