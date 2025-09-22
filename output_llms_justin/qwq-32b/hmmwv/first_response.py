import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Create the Chrono system with NSC contact method
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  # Gravity

# Initialize Irrlicht visualization
vis = irr.ChIrrApp(sys, 'HMMWV Simulation', irr.CHIRRRESIZABLE)
vis.SetCameraPosition(chrono.ChVectorD(15, 15, 15))
vis.SetCameraTarget(chrono.ChVectorD(0, 0, 0))

# Create and initialize rigid terrain
terrain = veh.ChRigidTerrain(sys)
patch = veh.ChTerrainPatchRectangle(
    chrono.ChVectorD(20, 20, 0.1),  # Dimensions (length, width, resolution)
    chrono.ChVectorD(0, 0, 0),       # Position
    chrono.ChVectorD(0, 0, 0),       # Rotation axis (no rotation)
    chrono.ChMaterialSurface(),      # Material
    'textures/grass.jpg'             # Texture (adjust path as needed)
)
terrain.Initialize(patch, True)  # Add collision model to the system
vis.AddAsset(patch.GetVisualShape())  # Add terrain visual asset

# Create and initialize HMMWV vehicle
hmmwv = veh.HMMWV_Vehicle()
hmmwv.Initialize(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT),
    veh.ChTireModelType.TMEASY,      # TMEASY tire model
    veh.ChVisualSystemType_PRIMITIVES  # Primitive visualization
)
sys.Add(hmmwv.GetSystem())  # Add vehicle system to the Chrono system

# Create interactive driver
driver = irr.ChIrrGuiDriver()
driver.Initialize(vis, hmmwv.GetDriverInputs())

# Add vehicle visualization to Irrlicht app
vis.Add(hmmwv.GetVehicle())

# Simulation settings
vis.SetTimestep(0.02)  # 50 Hz
vis.SetTryRealtime(True)

# Main simulation loop
while vis.Run():
    # Update driver inputs based on user input
    driver.Synchronize(sys.GetChTime())
    
    # Advance simulation step
    hmmwv.Synchronize()
    hmmwv.Advance()
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()