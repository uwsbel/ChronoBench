import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def run_simulation():
    
    sys = chrono.ChSystemNSC()
    veh.SetVehicleDataFile(veh.GetDataFile("vehicle/json/feda.json"))
    vehicle = veh.ChVehicleSystem(sys, "FEDA", veh.VehicleInitMode.RIGID_TERRAIN)
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetTireModelType(veh.TireModelType_TMEASY)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    init_pos = chrono.ChVector3d(0, 0, 0.5)
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
    vehicle.Initialize(init_pos, init_rot)
    vehicle.SetChassisFixed(False)
    vehicle.EnableRealtime(True)

    
    for axle in vehicle.GetAxles():
        for side in [veh.VehicleSide_LEFT, veh.VehicleSide_RIGHT]:
            tire = veh.ChTMeasyTire(sys)
            tire.SetName("TMeasy Tire")
            tire.SetStepsize(sys.GetStep())
            tire.Initialize(axle, side)
            vehicle.AddTire(axle, side, tire)

    
    terrain = veh.ChRigidTerrain(sys)
    patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -1), chrono.QUNIT), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("vehicle/terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("FEDA Vehicle Simulation")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, -6, 1.5), chrono.ChVector3d(0, 0, 1.2))
    vis.AddTypicalLights()

    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetVehicle(vehicle)
    driver.SetSteeringDelta(0.05)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.05)
    driver.Initialize()

    
    step_size = 1.0 / 50.0
    real_time = 0.0
    time = 0.0

    while vis.Run():
        
        driver.UpdateInputs()
        driver.SynchronizeInputs(time)

        
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        sys.DoStepDynamics(step_size)
        time += step_size

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        real_time = vis.GetTime() - time
        if real_time < step_size:
            chrono.ChSleep(max(0.0, step_size - real_time))

    vis.Shutdown()

if __name__ == "__main__":
    run_simulation()