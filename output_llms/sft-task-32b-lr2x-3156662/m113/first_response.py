import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.m113 as m113

def run_simulation():
    # Initialize Chrono system
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # Create M113 vehicle
    vehicle = m113.M113()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(chrono.VisualizationType_MESH)

    # Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(chrono.ChCoordsysd(), 100.0, 100.0)
    patch.SetFriction(0.9)
    patch.SetRestitution(0.01)
    terrain.Initialize()

    # Initialize driver
    vis = chrono.ChVisualSystemIrrlicht()
    driver = veh.ChInteractiveDriverIRR(vehicle, vis, "M113 Driver")
    driver.SetSteeringDelta(0.03)
    driver.SetThrottleDelta(0.03)
    driver.SetBrakingDelta(0.03)

    # Set up visualization
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono M113 Simulation")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, -6, 1.5), chrono.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.EnableShadows(True)

    # Simulation loop parameters
    step_size = 1e-3
    max_it = 10000
    it = 0

    while vis.Run() and it < max_it:
        time = system.GetChTime()
        driver.Update()
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, None)

        system.DoStepDynamics(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(time, step_size)
        vis.Advance()

        it += 1

    vis.Close()

if __name__ == "__main__":
    run_simulation()