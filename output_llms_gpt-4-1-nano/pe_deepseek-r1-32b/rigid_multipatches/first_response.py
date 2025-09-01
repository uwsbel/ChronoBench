importchrono
import math
import numpy as np
from pychrono import ChSystemNSC, ChCollisionModel, ChContactMaterialNSC
from pychrono.vehicle import HMMWV, RigidTerrain, VehicleEngineType, VehicleDrivetrainType
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChColor

# Initialize the simulation system
system = ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Visualization setup
vis = ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV Simulation")
vis.AddCamera(chrono.ChVector3d(10, 10, 10))
vis.AddTypicalLights()

# Vehicle initialization
vehicle = HMMWV(system, True, True)
vehicle.SetEngineType(VehicleEngineType.Diesel)
vehicle.SetDrivetrainType(VehicleDrivetrainType.FWD)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.Initialize()

# Terrain creation
terrain = RigidTerrain(system)
patch_mat = chrono.ChMaterialSurface()
patch_mat.SetFriction(0.8)

# Flat patch with texture
patch1 = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
patch1.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))

# Another flat patch with different texture
patch2 = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(50, 0, 0), chrono.QUNIT), 100, 100)
patch2.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"))

# Mesh-based bump patch
bump_mesh = chrono.ChTriangleMesh()
bump_mesh.Load(chrono.GetChronoDataFile("meshes/bump.obj"))
patch3 = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 50, 0), chrono.QUNIT), 100, 100)
patch3.SetMesh(bump_mesh)

# Heightmap-based patch
heightmap = np.random.rand(100, 100) * 5  # Random heightmap
patch4 = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, -50, 0), chrono.QUNIT), 100, 100)
patch4.SetHeightmap(heightmap)

terrain.Initialize()

# Driver system
driver = vehicle.GetDriver()
driver.SetType(chrono.ChDriverType.INTERACTIVE)

# Simulation parameters
timestep = 0.01
time = 0

# Main simulation loop
while vis.Run():
    vis.BeginScene()
    
    # Update driver inputs
    driver.SetThrottle(chrono.GetInput().GetKeyDown(chrono.KEY_UP) - chrono.GetInput().GetKeyDown(chrono.KEY_DOWN))
    driver.SetSteering(chrono.GetInput().GetKeyDown(chrono.KEY_RIGHT) - chrono.GetInput().GetKeyDown(chrono.KEY_LEFT))
    driver.SetBraking(chrono.GetInput().GetKeyDown(chrono.KEY_DOWN))
    
    # Update vehicle and terrain
    vehicle.Update(time)
    terrain.Update(time)
    
    # Render scene
    vis.Render()
    vis.EndScene()
    
    # Advance simulation
    system.DoStepDynamics(timestep)
    time += timestep
    
    # Print vehicle speed
    print(f"Time: {time:.2f}s, Speed: {vehicle.GetSpeed():.2f} m/s")