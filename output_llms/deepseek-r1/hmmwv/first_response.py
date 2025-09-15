#!/usr/bin/env python
# coding: utf-8

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math

def main():
    # =============================================================================
    # Initialize simulation
    # =============================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    system.SetSolverMaxIterations(150)

    # =============================================================================
    # Create terrain
    # =============================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # =============================================================================
    # Create and initialize HMMWV vehicle
    # =============================================================================
    hmmwv = veh.HMMWV_Full(system)
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(0.001)
    hmmwv.Initialize()

    # Visualization settings
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    # =============================================================================
    # Create interactive driver system
    # =============================================================================
    driver = veh.ChIrrGuiDriver(hmmwv.GetVehicle())

    # Set driver parameters
    steering_time = 1.0    # Time to turn from max left to max right (s)
    driver.SetSteeringDelta(0.02 / steering_time)
    throttle_time = 1.0    # Time to go from 0 to max throttle (s)
    driver.SetThrottleDelta(0.02 / throttle_time)
    braking_time = 0.3     # Time to go from 0 to max braking (s)
    driver.SetBrakingDelta(0.02 / braking_time)

    driver.Initialize()

    # =============================================================================
    # Create Irrlicht visualization
    # =============================================================================
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowTitle('HMMWV Simulation')
    vis.SetWindowSize(1280, 720)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 8, 1.5), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 0, 8), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 35, 512)

    # Attach vehicle systems to visualization
    vis.AttachVehicle(hmmwv.GetVehicle())

    # =============================================================================
    # Simulation loop
    # =============================================================================
    step_size = 0.002
    frame_interval = 0.02  # 50 FPS (1/50 = 0.02 seconds)
    time = 0
    frame_number = 0

    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        
        # Update vehicle systems
        hmmwv.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        # Advance dynamics
        system.DoStepDynamics(step_size)
        
        # Update visualization
        if time >= frame_number * frame_interval:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            frame_number += 1

if __name__ == "__main__":
    main()