import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

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
    vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    ofile = open('output/driving_data.txt', 'w')
    output = veh.ChVehicleOutput(veh.ChVehicleOutput.FORMAT_TAB_SEPARATED, ofile)
    output.AddOutput(veh.ChVehicleOutput.OutputType_DRIVING_DATA)
    output.Initialize()

    
    steering_force = 0.0
    driving_style = veh.DrivingStyle_NONE
    vehicle.GetVehicle().EnableRealtime(True)

    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        
        if (time >= 3):
            break

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = vehicle.GetDriverInputs()

        
        
        
        
        

        
        
        
        
        

        
        output.Update(time, vehicle)

        
        if (driving_style == veh.DrivingStyle_KEYBOARD):
            steering_force = driver_inputs-steering_force
            steering = min(max(steering_force, -1.0), 1.0)
            vehicle.SynchronizeSteering(steering)
        elif (driving_style == veh.DrivingStyle_NONE):
            
            pass

    return 0





veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

main()