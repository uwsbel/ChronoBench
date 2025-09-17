#!/usr/bin/env python3
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # 1. Initialize system and set contact method
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))

    # 2. Create FEDA vehicle
    vehicle = veh.FEDA(system)
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.Q_from_AngZ(0)))
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()

    # 3. Set visualization modes
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # 4. Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceSMC()
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # 5. Set up Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("FEDA Vehicle Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(6, 6, 1.5), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(10, 10, 8), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 40, 512)
    vis.EnableShadows()

    # 6. Create interactive driver and set camera tracking
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # 7. Configure vehicle system
    vehicle.GetVehicle().SetChaseCamera(chrono.ChVectorD(0, 0, 1.5), 10, 0.5)
    vehicle.GetVehicle().SetChaseCameraState(True)

    # 8. Simulation loop at 50 FPS
    step_size = 0.002
    render_step = 1.0 / 50  # 50 FPS

    realtime_timer = chrono.ChRealtimeStepTimer()
    time = 0
    frame_number = 0

    while vis.Run():
        time = system.GetChTime()

        # Update visualization and inputs
        vis.BeginScene()
        vis.Render()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        vis.EndScene()

        # Advance dynamics
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)

        # Enforce real-time synchronization
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()