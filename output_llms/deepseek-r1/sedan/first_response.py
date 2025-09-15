#!/usr/bin/env python3
# PyChrono simulation of BMW E90 Sedan driving on rigid terrain

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # ==========================================================================
    # Initialize PyChrono environment
    # ==========================================================================
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(veh.GetDataPath() + "vehicle/")
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
    system.SetSolverMaxIterations(150)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

    # ==========================================================================
    # Create ground and terrain
    # ==========================================================================
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetCollide(True)
    
    # Create collision geometry for ground
    ground_shape = chrono.ChCollisionModel()
    ground_shape.ClearModel()
    ground_shape.AddBox(100, 100, 0.2, chrono.ChVectorD(0, 0, -0.2))
    ground_shape.BuildModel()
    ground.SetCollisionModel(ground_shape)
    
    # Add ground to system
    system.Add(ground)
    
    # Add visual assets to ground
    ground_vis = chrono.ChBoxShape()
    ground_vis.GetBoxGeometry().Size = chrono.ChVectorD(100, 100, 0.2)
    ground_vis.SetColor(chrono.ChColor(0.6, 0.6, 0.5))
    ground.AddAsset(ground_vis)

    # Add textured logo/pattern on ground
    texture = chrono.ChTexture()
    texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddAsset(texture)

    # ==========================================================================
    # Create and initialize BMW E90 vehicle
    # ==========================================================================
    vehicle = veh.Sedan(system)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.Initialize()

    # Set visualization mode for chassis and wheels
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # ==========================================================================
    # Create interactive driver system
    # ==========================================================================
    driver = veh.ChInteractiveDriverIRR(vehicle)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # ==========================================================================
    # Create Irrlicht visualization system
    # ==========================================================================
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("BMW E90 Sedan Simulation")
    vis.Initialize()
    
    # Configure chase camera
    vis.AddTypicalLights(
        irr.vector3df(-50, -50, 80),  # Left light position
        irr.vector3df(50, 50, 80)     # Right light position
    )
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, -8, 1.5), chrono.ChVectorD(0, 0, 0.5))
    
    # ==========================================================================
    # Simulation loop
    # ==========================================================================
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()
    
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Update vehicle systems
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        
        # Advance simulation
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        driver.Advance(step_size)
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()