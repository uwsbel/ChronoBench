import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math



def main():
    

    
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    
    terrain = chrono.ChTerrain(sys)
    topo = terrain.GetTopo()
    topo.LoadHeightField(200, 100, lambda x,y: 0.1*chrono.ChFunctionSine(x*0.015).GetVal(y*0.03),  
                          lambda x,y: 2,  
                          lambda x,y: 1,  
                          False,          
                          True            
                          )
    terrain.SetTexture(veh.GetDataFile('terrain/textures/tile4.jpg'), 200, 200)
    terrain.Initialize()
    terrain.SetFixed(True)

    
    fwd = veh.FWD(sys)
    fwd.SetName("FWD")
    fwd.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, 1.0), chrono.QUNIT))
    fwd.SetEngineType(veh.EngineModelType_SIMPLE)
    fwd.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    fwd.SetDriveType(veh.DrivelineTypeWV_AWD)
    fwd.SetTireType(veh.TireModelType_RIGID)
    fwd.Initialize()
    fwd.SetContactMethod(chrono.ChContactMethod_NSC)

    
    vehicle = veh.Turtlebot(fwd)
    vehicle.SetName("Turtlebot")
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -0.5, 1.0), chrono.QUNIT))
    vehicle.SetEngineShafts(fwd.GetEngine().GetMotorOutputs()[0].GetShafts()[0])
    vehicle.SetTransmissionShafts(fwd.GetTransmission().GetOutputShaft())
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
    vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Turtlebot')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0,0.0,1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())

    
    if (not vis.GetSystem().Run()):
        return

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    
    time = 0
    tick_time = 0
    t_end = 100
    time_left = 1.0 / 30.0  
    while (time < t_end) :
        time = vehicle.GetSystem().GetChTime()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

    return








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 1e-3

main()