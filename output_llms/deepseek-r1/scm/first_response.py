#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Import PyChrono modules
import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

def main():
    # ==========================================================================
    # Initialize simulation environment
    # ==========================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(veh.GetDataPath() + 'vehicle/')
    
    # Create Chrono system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # ==========================================================================
    # Create SCM deformable terrain
    # ==========================================================================
    terrain = veh.SCMDeformableTerrain(system)
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, -0.5), chrono.Q_from_AngX(-math.pi/2)))
    
    # Soil parameters (custom values)
    soil_params = veh.SoilParametersCallback()
    soil_params.m_Kn = 2e6   # Normal stiffness
    soil_params.m_Kt = 1e6   # Tangential stiffness
    soil_params.m_n = 1.1    # Power exponent
    soil_params.m_moist_cohesion = 0.0
    soil_params.m_moist_friction = 30.0
    soil_params.m_dry_cohesion = 0.0
    soil_params.m_dry_friction = 35.0
    terrain.SetSoilParameters(soil_params)
    
    # Terrain visualization
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.15)
    terrain.SetColorTableMin(0.0)
    terrain.SetColorTableMax(0.15)
    
    # Moving patch configuration
    patch_size = 5.0
    terrain.Initialize(0.0, 0.0, patch_size, patch_size, 0.04)
    
    # ==========================================================================
    # Create and initialize HMMWV vehicle
    # ==========================================================================
    # Create vehicle system
    hmmwv = veh.HMMWV_Full(system)
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.Q_from_AngZ(0)))
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.Initialize()
    
    # Set visualization parameters
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # ==========================================================================
    # Create Irrlicht visualization
    # ==========================================================================
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV on SCM Terrain')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 8, 1.5), chrono.ChVectorD(0, 0, 0.5))
    vis.AddTypicalLights()
    
    # ==========================================================================
    # Create interactive driver
    # ==========================================================================
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()
    
    # ==========================================================================
    # Simulation loop
    # ==========================================================================
    step_size = 0.002
    render_step = 1.0 / 50  # 50 FPS
    
    realtime_timer = chrono.ChRealtimeStepTimer()
    time = 0
    frame_number = 0
    
    while vis.Run():
        time = system.GetChTime()
        
        # Update moving patch to follow vehicle
        chassis_pos = hmmwv.GetVehicle().GetChassisBody().GetPos()
        terrain.SetPatchCenter(chassis_pos.x, chassis_pos.y)
        
        # Render scene
        if frame_number % int(render_step / step_size) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Update systems
        driver.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)
        
        # Advance simulation
        hmmwv.Advance(step_size)
        terrain.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Real-time stepping
        realtime_timer.Spin(step_size)
        frame_number += 1

if __name__ == "__main__":
    main()