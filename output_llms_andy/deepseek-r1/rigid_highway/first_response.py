#!/usr/bin/env python
# coding: utf-8

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # ==========================================================================
    # Initialize Simulation Environment
    # ==========================================================================
    
    # Set path to Chrono data directory
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # Create Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    
    # Set contact method (NSC - Non-Smooth Contacts)
    contact_method = chrono.ChContactMethod_NSC
    
    # ==========================================================================
    # Create HMMWV Vehicle
    # ==========================================================================
    
    # Initialize vehicle parameters
    init_pos = chrono.ChVectorD(0, 0, 0.5)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    tire_type = veh.TireModelType_TMEASY
    
    # Create full HMMWV vehicle
    hmmwv = veh.HMMWV_Full(system)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    hmmwv.SetTireType(tire_type)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitFwdVel(0.0)
    
    # Configure visualization
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    
    # Initialize vehicle
    hmmwv.Initialize()
    
    # ==========================================================================
    # Create Custom Terrain
    # ==========================================================================
    
    # Create terrain body
    terrain = chrono.ChBody()
    terrain.SetPos(chrono.ChVectorD(0, 0, 0))
    terrain.SetBodyFixed(True)
    system.Add(terrain)
    
    # Load collision mesh
    collision_mesh = chrono.ChTriangleMeshConnected()
    collision_mesh.LoadWavefrontMesh("Highway_col.obj")
    
    # Add collision shape
    collision_shape = chrono.ChTriangleMeshShape()
    collision_shape.SetMesh(collision_mesh)
    collision_shape.SetName("collision_mesh")
    collision_shape.SetStatic(True)
    terrain.AddAsset(collision_shape)
    
    # Add visual mesh
    visual_mesh = chrono.ChObjFileShape()
    visual_mesh.SetFilename("Highway_vis.obj")
    terrain.AddAsset(visual_mesh)
    
    # Add collision geometry
    terrain.GetCollisionModel().ClearModel()
    terrain.GetCollisionModel().AddTriangleMesh(collision_mesh, True, False)
    terrain.GetCollisionModel().BuildModel()
    terrain.SetCollide(True)
    
    # ==========================================================================
    # Create Interactive Driver
    # ==========================================================================
    
    driver = veh.ChIrrGuiDriver()
    
    # ==========================================================================
    # Setup Irrlicht Visualization
    # ==========================================================================
    
    # Create Irrlicht application
    vis = veh.ChWheeledVehicleIrrApp(hmmwv.GetVehicle(), "HMMWV Simulation")
    vis.SetSkyBox()
    vis.AddTypicalLights()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    
    # Attach driver system
    vis.SetDriver(driver)
    
    # Initialize visualization
    vis.Initialize()
    
    # ==========================================================================
    # Simulation Loop
    # ==========================================================================
    
    # Simulation parameters
    step_size = 0.02  # 50 FPS = 0.02s per frame
    realtime_step = True
    
    while vis.Run():
        # Get simulation time
        time = system.GetChTime()
        
        # Draw scene
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Update vehicle systems
        hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)
        
        # Advance simulation
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.Advance(step_size)
        
        # Maintain real-time stepping
        if realtime_step:
            vis.SetRealtimeStep(step_size)

if __name__ == "__main__":
    main()