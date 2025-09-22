#!/usr/bin/env python3
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # ==========================================================================
    # Initialize PyChrono environment
    # ==========================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(veh.GetDataPath())

    # Create physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # ==========================================================================
    # Create terrain
    # ==========================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # ==========================================================================
    # Create and initialize Kraz vehicle
    # ==========================================================================
    vehicle = veh.Kraz(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0)))
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.Initialize()

    # Set vehicle subsystems
    tire_model = veh.TMeasyTire()
    vehicle.SetTireType(tire_model)
    vehicle.SetChassisFixed(False)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # ==========================================================================
    # Create driver system
    # ==========================================================================
    driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())

    # Set driver parameters
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    # ==========================================================================
    # Create visualization system
    # ==========================================================================
    vis = irr.ChIrrApp(system, "Kraz Vehicle Simulation", irr.dimension2du(1280, 720))
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 2, 0), chrono.ChVectorD(0, 0, 0))
    vis.AddLightWithShadow(chrono.ChVectorD(20, 20, 20), chrono.ChVectorD(0, 0, 0), 
                           50, 5, 50, 35, 512, chrono.ChColor(0.8, 0.8, 0.8))
    vis.AddLight(chrono.ChVectorD(-20, -20, 0), 6, chrono.ChColor(0.7, 0.7, 0.7))
    vis.EnableShadows()

    vehicle.SetIrrlichtRenderer(vis.GetDevice())
    vis.AssetBindAll()
    vis.AssetUpdateAll()

    # ==========================================================================
    # Simulation loop
    # ==========================================================================
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        time = system.GetChTime()

        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)

        # Update vehicle systems
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)

        # Advance simulation
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)

        # Maintain real-time execution
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()