import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.irrlicht as veh_irr

def run_citybus_simulation():
    # Initialize Chrono and vehicle modules
    veh.SetVehicleDataPath(veh.GetChronoDataPath() + "vehicle/")
    
    # Create the CityBus vehicle
    vehicle = veh.CityBus()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.SetTireStepSize(1e-3)
    vehicle.Initialize()
    
    # Set visualization types for vehicle components
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    
    # Create rigid terrain with custom texture
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch = terrain.AddPatch(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 20.0, 20.0)
    patch.SetTexture(veh.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()
    
    # Set up Irrlicht visualization
    vis = veh_irr.ChWheeledVehicleIrrApp(vehicle, "PyChrono CityBus Simulation")
    vis.SetSkyBox()
    vis.SetCamera(chrono.ChVector3d(0, -8, 2.5), chrono.ChVector3d(0, 0, 1.5))
    vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.5), 6.0, 0.5)
    vis.AddLightDirectional()
    vis.AddLightPoint(chrono.ChVector3d(10, -10, 10), 100)
    vis.AddLightPoint(chrono.ChVector3d(-10, -10, 10), 100)
    vis.SetShadowMapEnabled(True, 1024, 512, 3.5, 512, 45)
    
    # Create interactive driver system
    driver = veh_irr.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.05)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    # Simulation loop parameters
    step = 0.02  # 50 FPS
    time = 0
    max_time = 30
    
    while vis.Run():
        # Update driver inputs and synchronize vehicle
        time = vis.GetTime()
        driver.Update(step)
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        vehicle.Advance(step)
        vis.NextFrame()
        
        # Exit condition
        if time >= max_time:
            break

if __name__ == "__main__":
    run_citybus_simulation()