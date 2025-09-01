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
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -1.5, 1.0), chrono.ChQuaterniond(1, 0, 0, 0)))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(step_size)
    vehicle.SetInitFwdVel(0.0)
    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
    vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Differential Steering')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    
    out = open('steering_data.txt', 'w')

    
    vehicle.EnableRealtime(True)

    while vis.Run() :
        time = vehicle.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = vehicle.GetDriverInputs()

        
        driver_inputs_valid = (driver_inputs-steering_inputs).squared_magnitude < 1e-6
        vehicle.Synchronize(time)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        
        steering_error = vehicle.GetSteeringError(chrono.ChVector3d(1, 0, 0))
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

        if driver_inputs_valid:
            out.write( "%f %f %f %f\n" % (time, driver_inputs.steering, driver_inputs.throttle, driver_inputs.braking))
            out.flush()

        if (time < 2.0):
            
            y = vehicle.GetChassis().GetPos().y
            if (y > 0.0):
                vehicle.GetChassis().SetPos(chrono.ChVector3d(vehicle.GetChassis().GetPos().x, 0.0, vehicle.GetChassis().GetPos().z))

    return 0

steering_inputs = veh.DriverInputs()
steering_inputs.steering = 0.0
steering_inputs.throttle = 0.0
steering_inputs.braking = 0.0

main()