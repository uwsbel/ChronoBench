import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

def main():
    

    step_size = 0.005

    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    
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

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Differential Steering')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
    vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    ofile = open('steering_angles.txt', 'w')

    
    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        
        if (time >= 3):
            break

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = vehicle.GetDriverInputs()

        
        driver_inputs_valid = False
        if vis.Run() :
            driver_inputs_valid = True
            steering = driver_inputs.m_steering
            braking = driver_inputs.m_braking
            accelerating = driver_inputs.m_accelerating

        vehicle.Synchronize(time)
        terrain.Synchronize(time)
        if driver_inputs_valid:
            vis.Synchronize(time, driver_inputs)
            ofile.write("Time = " + str(time) + "\nSteering = " + str(steering) + "\nBraking = " + str(braking) + "\nAccelerating = " + str(accelerating) + "\n\n")

        
        driver_inputs_valid = False
        if vis.Run() :
            driver_inputs_valid = True
            steering = driver_inputs.m_steering
            braking = driver_inputs.m_braking
            accelerating = driver_inputs.m_accelerating

        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        if driver_inputs_valid:
            vis.Advance(step_size, driver_inputs)
            ofile.write("Steering = " + str(steering) + "\nBraking = " + str(braking) + "\nAccelerating = " + str(accelerating) + "\n\n")

        ofile.flush()

main()