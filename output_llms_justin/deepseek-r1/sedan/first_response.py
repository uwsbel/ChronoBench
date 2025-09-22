#!/usr/bin/env python
# coding: utf-8

# BMW E90 Sedan Simulation with PyChrono
import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# =============================================================================
# Main Simulation Parameters
# =============================================================================
TIME_STEP = 0.0025  # Simulation time step
VISUALIZATION_STEP = 1.0 / 60  # Visualization update rate
TERRAIN_LENGTH = 100.0  # Terrain length (X direction)
TERRAIN_WIDTH = 20.0   # Terrain width (Y direction)

# =============================================================================
# Initialize Chrono System
# =============================================================================
chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# =============================================================================
# Create Terrain
# =============================================================================
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetCompliance(1e-6)

patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(0, 0, 0), 
                         chrono.ChVectorD(0, 0, 1), 
                         TERRAIN_LENGTH, 
                         TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# =============================================================================
# Create BMW E90 Vehicle
# =============================================================================
bmw = veh.BMW_E90(system)
bmw.SetContactMethod(chrono.ChContactMethod_NSC)
bmw.SetChassisFixed(False)
bmw.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.7), chrono.QUNIT))
bmw.SetTireType(veh.TireModelType_TMEASY)
bmw.Initialize()

# Set vehicle visualization mode
bmw.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
bmw.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bmw.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bmw.SetWheelVisualizationType(veh.VisualizationType_MESH)
bmw.SetTireVisualizationType(veh.VisualizationType_MESH)

# =============================================================================
# Create Driver System
# =============================================================================
driver = veh.ChIrrGuiDriver(bmw.GetVehicle())

# Set driver parameters
driver.SetThrottleThreshold(0.2)
driver.SetGains(2.0, 5.0, 0.0)
driver.Initialize()

# =============================================================================
# Setup Irrlicht Visualization
# =============================================================================
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('BMW E90 Sedan Simulation')
vis.Initialize()

# Add visualization assets
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 8, 1.0), chrono.ChVectorD(5, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, 10), chrono.ChVectorD(0, 0, 0), 30, 1, 20, 60, 512)

# Enable chase-cam and grid
vis.EnableShadows()
vis.EnableGrid(0.5, 0.5, 40, 40, 
               chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.1), 
               chrono.Q_from_AngX(chrono.CH_C_PI_2)),
               irr.SColor(255, 80, 100, 100))

# Attach vehicle systems to visualizer
vis.AttachVehicle(bmw.GetVehicle())

# =============================================================================
# Simulation Loop
# =============================================================================
bmw.GetVehicle().EnableRealtime(True)

while vis.Run():
    time = system.GetChTime()
    
    # Update driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    # Update vehicle systems
    bmw.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    driver.Advance(TIME_STEP)
    bmw.Advance(TIME_STEP)
    terrain.Advance(TIME_STEP)
    system.DoStepDynamics(TIME_STEP)
    
    # Update visualization
    if system.GetChTime() % VISUALIZATION_STEP < TIME_STEP:
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChFrameD(), 1.0)
        vis.EndScene()

        # Display stats
        vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)
        vis.Advance(TIME_STEP)