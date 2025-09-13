#!/usr/bin/env python3
import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# =============================================================================
# Main simulation function
# =============================================================================
def main():
    # 1. Initialize PyChrono environment
    chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # 2. Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), 
                            chrono.ChVectorD(0, 0, 1), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # 3. Create and initialize MAN truck
    truck = veh.MAN_10t()
    truck.SetContactMethod(chrono.ChContactMethod_NSC)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0), chrono.QUNIT))
    truck.SetTireType(veh.TireModelType_TMEASY)
    truck.Initialize()

    # Configure visualization
    truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)

    # 4. Create interactive driver system
    driver = veh.InteractiveDriverIRR(truck.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # 5. Create Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('MAN 10t Truck Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, -5), chrono.ChVectorD(0, 0, 0), 
                          50, 5, 50, 35, 512, chrono.ChColor(0.8, 0.8, 1.0))

    # 6. Simulation loop
    step_size = 0.005
    render_step = 1.0 / 50  # 50 FPS

    truck.GetVehicle().EnableRealtime(True)
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Update vehicle systems
        truck.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        # Advance simulation
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChVectorD(0,0,0), chrono.QUNIT, 2)  # Reference frame
        vis.EndScene()
        
        # Draw steering wheel and controls
        driver.DrawHUD(True, True, True, 120, 40, 12, 0.5, 0.5)

# =============================================================================
# Entry point
# =============================================================================
if __name__ == "__main__":
    main()