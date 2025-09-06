import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.tire as tire
import pychrono.irrlicht as chronoirr

# Initialize the simulation system with NSC contact method
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity vector

# Setup visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.AttachSystem(my_system)
vis.SetCamera(chronoirr.ChVectorD(0, 10, 30))  # Camera position
vis.AddTypicalLights()
vis.AddSkyBox()
vis.Initialize()

# Create terrain
terrain = veh.RigidTerrain(my_system)
ground_mat = chrono.ChMaterialSurfaceNSC()  # Default friction etc.
patch = terrain.AddPatch(ground_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)  # 100x100m terrain
patch.texture_path = "path/to/texture.jpg"  # Replace with actual texture path
terrain.Initialize()

# Initialize Gator vehicle
gator = veh.Gator(my_system)
gator.SetContactMethod(chrono.ChSystemContactMethod.NSC)  # Match system contact method
gator.SetTireType(tire.TMeasyTire())  # TMEASY tire model
gator.SetMeshVisuals(True)  # Use detailed mesh visuals

# Position and orientation (Y=0.5 to rest on ground)
init_pos = chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT)  # No rotation
gator.Initialize(init_pos)

# Setup interactive driver
driver = veh.InteractiveDriver()
gator.SetDriver(driver)

# Simulation loop parameters
time_step = 0.02  # 50 FPS simulation step

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Update driver inputs (steering, throttle, brake)
    driver.Synchronize(my_system.GetChTime())
    
    # Advance simulation
    my_system.DoStepDynamics(time_step)

# Cleanup (not strictly needed as it's at end of script)
vis.Close()