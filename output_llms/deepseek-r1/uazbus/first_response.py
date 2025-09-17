#!/usr/bin/env python
# coding: utf-8

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# =============================================================================
# Main simulation function
# =============================================================================
def main():
    # 1. Initialize PyChrono environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(100)
    system.SetMaxPenetrationRecoverySpeed(4.0)

    # 2. Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)  # Friction coefficient
    patch_mat.SetRestitution(0.01)  # Restitution coefficient
    terrain.Initialize(chrono.ChVectorD(0, 0, 0), 
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                       200, 200, 0.1, patch_mat)

    # 3. Create and initialize UAZBUS vehicle
    vehicle = veh.UAZBUS()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 2), chrono.Q_from_AngZ(0)))
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()

    # Set vehicle subsystems parameters
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # 4. Create driver system
    driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())

    # 5. Initialize Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("UAZBUS Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 3, 4), chrono.ChVectorD(0, 0, 0.5))
    vis.AddLightWithShadow(chrono.ChVectorD(20, 20, 20), chrono.ChVectorD(0, 0, 0), 50, 10, 50, 2)
    vis.AddTypicalLights()
    vis.EnableShadows()

    # Bind visualization assets
    vehicle.GetVehicle().SetVisualSystem(vis)

    # 6. Simulation loop
    step_size = 0.02
    realtime_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        time = system.GetChTime()

        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)

        # Update vehicle systems
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)

        # Advance simulation
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

        # Update visualization
        vis.Synchronize(time, driver_inputs)
        vehicle.Advance(step_size)

# =============================================================================
# Entry point
# =============================================================================
if __name__ == '__main__':
    main()