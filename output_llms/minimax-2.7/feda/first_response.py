#!/usr/bin/env python3
"""
PyChrono FEDA Vehicle Simulation on Rigid Terrain
Using Irrlicht for 3D visualization
"""

# ============================================================================
# IMPORTS
# ============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
from pychrono.vehicle import FEDAParams
from pychrono.vehicle import FEDAVehicle
from pychrono.vehicle import RigidTerrain
from pychrono.vehicle import ChWheeledVehicleVisualSystemIrrlicht
from pychrono.vehicle import ChDriver
from pychrono.vehicle import ChPathFollowerDriver
import math

# ============================================================================
# SIMULATION PARAMETERS
# ============================================================================

# Time settings
SIMULATION_FPS = 50
SIMULATION_DURATION = 100.0  # seconds
VISUALIZATION_STEP = 1.0 / SIMULATION_FPS

# Vehicle parameters
VEHICLE_INITIAL_POS = chrono.ChVector3d(0, 0.5, 0)
VEHICLE_INITIAL_ROT = chrono.ChQuatFromAngleZ(0)  # No initial rotation
VEHICLE_MASS = 1500.0  # kg

# Terrain parameters
TERRAIN_LENGTH = 200.0  # meters
TERRAIN_WIDTH = 100.0   # meters
TERRAIN_WALL_HEIGHT = 1.0

# Camera parameters
CAMERA_OFFSET = chrono.ChVector3d(-8.0, 3.0, 0.0)  # Behind and above vehicle

# ============================================================================
# CREATE PHYSICS SYSTEM AND WORLD
# ============================================================================

# Create the physical system
system = chrono.ChSystemNSC()
system.SetNumThreads(4)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)
system.Set timestep(0.002)

# Create the world
my_world = chrono.ChWorld()
system.Add(my_world)

print("PyChrono environment initialized successfully.")
print(f"Physics system created with {system.GetNumThreads()} threads.")

# ============================================================================
# CREATE TERRAIN
# ============================================================================

# Define terrain material (friction and restitution)
terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.8)
terrain_material.SetRestitution(0.1)
terrain_material.SetPlasticCompliance(0.001)
terrain_material.SetYieldStress(1e7)

# Create rigid terrain with custom texture
print("Creating rigid terrain...")

# Create terrain body
terrain_body = chrono.ChBody()
terrain_body.SetIdentifier(-1)
terrain_body.SetName("Terrain")
terrain_body.SetMass(0)  # Static body
terrain_body.SetPos(chrono.ChVector3d(0, -TERRAIN_WALL_HEIGHT, 0))
terrain_body.EnableCollision(True)
terrain_body.SetMaterialSurface(terrain_material)

# Create terrain collision shape (box)
terrain_shape = chrono.ChCollisionShapeBox(
    chrono.ChVector3d(TERRAIN_LENGTH / 2, TERRAIN_WALL_HEIGHT, TERRAIN_WIDTH / 2)
)
terrain_body.AddCollisionShape(terrain_shape, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
system.AddBody(terrain_body)

# Create terrain visualization mesh
print("Creating terrain mesh with custom texture...")

# Generate procedural terrain texture coordinates
def generate_terrain_mesh(length, width, resolution):
    """Generate terrain mesh with custom texture coordinates"""
    vertices = []
    indices = []
    uvs = []
    
    step_x = length / resolution
    step_z = width / resolution
    
    # Generate vertices
    for i in range(resolution + 1):
        for j in range(resolution + 1):
            x = -length / 2 + i * step_x
            z = -width / 2 + j * step_z
            vertices.append(chrono.ChVector3d(x, 0, z))
            uvs.append(chrono.ChVector2d(i / resolution * 10, j / resolution * 10))
    
    # Generate triangle indices
    for i in range(resolution):
        for j in range(resolution):
            idx = i * (resolution + 1) + j
            indices.append(idx)
            indices.append(idx + resolution + 1)
            indices.append(idx + 1)
            indices.append(idx + 1)
            indices.append(idx + resolution + 1)
            indices.append(idx + resolution + 2)
    
    return vertices, indices, uvs

terrain_vertices, terrain_indices, terrain_uvs = generate_terrain_mesh(
    TERRAIN_LENGTH, TERRAIN_WIDTH, 50
)

# Create mesh asset for terrain
terrain_mesh = chrono.ChVisualShapeTriangleMesh()
mesh_geometry = chrono.ChTriangleMeshConnected()

for vertex in terrain_vertices:
    mesh_geometry.AddVertex(chrono.ChVector3d(vertex.x(), vertex.y(), vertex.z()))

for i in range(0, len(terrain_indices), 3):
    v1 = terrain_indices[i]
    v2 = terrain_indices[i + 1]
    v3 = terrain_indices[i + 2]
    triangle = chrono.ChTriangle(
        terrain_vertices[v1],
        terrain_vertices[v2],
        terrain_vertices[v3]
    )
    mesh_geometry.AddTriangle(triangle)

terrain_mesh.SetMesh(mesh_geometry)
terrain_mesh.SetBackfaceCull(True)

# Apply custom texture to terrain
texture_path = ""  # Empty for procedural texture
terrain_material_vis = chrono.ChVisualMaterial()
terrain_material_vis.SetAmbientColor(chrono.ChColor(0.4, 0.4, 0.4))
terrain_material_vis.SetDiffuseColor(chrono.ChColor(0.6, 0.6, 0.6))
terrain_material_vis.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))
terrain_material_vis.SetMetallic(0.0)
terrain_material_vis.SetRoughness(0.8)
terrain_mesh.material = terrain_material_vis

terrain_body.AddVisualShape(terrain_mesh, chrono.ChFrame(chrono.ChVector3d(0, -TERRAIN_WALL_HEIGHT, 0)))

# Create terrain walls (boundaries)
print("Creating terrain boundaries...")

def create_terrain_wall(position, size, rotation_angle=0):
    """Create a boundary wall for the terrain"""
    wall = chrono.ChBody()
    wall.SetIdentifier(-1)
    wall.SetName(f"TerrainWall_{position}")
    wall.SetMass(0)  # Static
    wall.EnableCollision(True)
    wall.SetMaterialSurface(terrain_material)
    
    wall.SetPos(position)
    wall.SetRot(chrono.ChQuatFromAngleY(rotation_angle))
    
    wall_shape = chrono.ChCollisionShapeBox(size)
    wall.AddCollisionShape(wall_shape, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
    
    # Add visual representation
    wall_mesh = chrono.ChVisualShapeBox(size[0], size[1], size[2])
    wall.AddVisualShape(wall_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
    
    system.AddBody(wall)
    return wall

# Create four boundary walls
wall_thickness = 0.5
wall_height = TERRAIN_WALL_HEIGHT + 2

create_terrain_wall(
    chrono.ChVector3d(0, wall_height / 2, -TERRAIN_WIDTH / 2 - wall_thickness / 2),
    chrono.ChVector3d(TERRAIN_LENGTH, wall_height, wall_thickness)
)
create_terrain_wall(
    chrono.ChVector3d(0, wall_height / 2, TERRAIN_WIDTH / 2 + wall_thickness / 2),
    chrono.ChVector3d(TERRAIN_LENGTH, wall_height, wall_thickness)
)
create_terrain_wall(
    chrono.ChVector3d(-TERRAIN_LENGTH / 2 - wall_thickness / 2, wall_height / 2, 0),
    chrono.ChVector3d(wall_thickness, wall_height, TERRAIN_WIDTH)
)
create_terrain_wall(
    chrono.ChVector3d(TERRAIN_LENGTH / 2 + wall_thickness / 2, wall_height / 2, 0),
    chrono.ChVector3d(wall_thickness, wall_height, TERRAIN_WIDTH)
)

print(f"Rigid terrain created: {TERRAIN_LENGTH}m x {TERRAIN_WIDTH}m")

# ============================================================================
# CREATE FEDA VEHICLE
# ============================================================================

print("Initializing FEDA vehicle...")

# Vehicle assembly parameters
vehicleAssembly = veh.ChVehicleAssembly()

# Create FEDA vehicle using vehicle parameters
# Using FEDA-specific vehicle configuration
feda_params = veh.FEDAParams()
feda_params.chassis_mass = VEHICLE_MASS
feda_params.chassis_dims = chrono.ChVector3d(4.0, 1.0, 2.0)
feda_params.wheelbase = 2.8
feda_params.front_suspension_height = 0.3
feda_params.rear_suspension_height = 0.3
feda_params.wheel_radius = 0.35
feda_params.wheel_width = 0.25
feda_params.max_steering_angle = 0.5  # radians

# Create vehicle
my_vehicle = veh.FEDAVehicle(
    system,
    VEHICLE_INITIAL_POS,
    VEHICLE_INITIAL_ROT,
    feda_params
)

print(f"Vehicle created at position: ({VEHICLE_INITIAL_POS.x()}, {VEHICLE_INITIAL_POS.y()}, {VEHICLE_INITIAL_POS.z()})")
print(f"Vehicle mass: {VEHICLE_MASS} kg")

# ============================================================================
# CONFIGURE VEHICLE VISUALIZATION (MESH TYPE)
# ============================================================================

print("Configuring mesh visualization for all vehicle parts...")

# Get vehicle subsystem for visualization
chassis = my_vehicle.GetChassis()
chassis.SetVisualType(chrono.ChVisualizationType_MESH)

# Create chassis mesh
chassis_mesh = chrono.ChVisualShapeBox(
    feda_params.chassis_dims.x(),
    feda_params.chassis_dims.y(),
    feda_params.chassis_dims.z()
)
chassis.AddVisualShape(chassis_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0.5, 0)))

# Configure chassis material
chassis_material = chrono.ChVisualMaterial()
chassis_material.SetDiffuseColor(chrono.ChColor(0.2, 0.4, 0.8))
chassis_material.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
chassis_material.SetMetallic(0.6)
chassis_material.SetRoughness(0.3)
chassis_mesh.material = chassis_material

# Configure wheels
wheels = my_vehicle.GetWheels()
for i, wheel in enumerate(wheels):
    wheel.SetVisualType(chrono.ChVisualizationType_MESH)
    
    # Create wheel mesh
    wheel_mesh = chrono.ChVisualShapeCylinder(
        feda_params.wheel_radius,
        feda_params.wheel_width
    )
    wheel.AddVisualShape(wheel_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
    
    # Configure wheel material
    wheel_material = chrono.ChVisualMaterial()
    wheel_material.SetDiffuseColor(chrono.ChColor(0.1, 0.1, 0.1))
    wheel_material.SetSpecularColor(chrono.ChColor(0.2, 0.2, 0.2))
    wheel_material.SetMetallic(0.8)
    wheel_material.SetRoughness(0.4)
    wheel_mesh.material = wheel_material

# Configure suspension components
suspensions = my_vehicle.GetSuspensions()
for suspension in suspensions:
    suspension.SetVisualType(chrono.ChVisualizationType_MESH)
    
    # Create suspension visualization
    suspension_mesh = chrono.ChVisualShapeBox(0.1, 0.3, 0.1)
    suspension.AddVisualShape(suspension_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0.15, 0)))

# ============================================================================
# CREATE INTERACTIVE DRIVER SYSTEM
# ============================================================================

print("Creating interactive driver system...")

# Create driver inputs
driver_inputs = veh.DriverInputs()

# Create keyboard driver for interactive control
keyboard_driver = veh.ChKeyboardDriver()

# Configure driver parameters
keyboard_driver.Initialize(system, my_vehicle)

print("Interactive driver system configured.")
print("Controls:")
print("  W/Up Arrow    - Accelerate (Throttle)")
print("  S/Down Arrow  - Brake")
print("  A/Left Arrow  - Steer Left")
print("  D/Right Arrow - Steer Right")
print("  Space         - Handbrake")
print("  R             - Reset vehicle")

# ============================================================================
# CREATE IRRLICHT VISUALIZATION
# ============================================================================

print("Initializing Irrlicht visualization...")

# Create Irrlicht application
application = chronoirr.ChIrrApp(
    system,
    "FEDA Vehicle Simulation",
    chronoirr.dimension2du(1280, 720),
    chronoirr.E_SHOW_TYPE.SHOW_ALL,
    chronoirr.E_DEVICE_TYPE.EIDT_DIRECT3D9
)

# Setup application
application.AddTypicalSky()
application.AddTypicalCamera(
    chronoirr.vector3df(CAMERA_OFFSET.x(), CAMERA_OFFSET.y(), CAMERA_OFFSET.z()),
    chronoirr.vector3df(0, 2, 0)
)
application.AddTypicalLights()
application.AddLightWithShadow(
    chronoirr.vector3df(10, 20, 10),
    chronoirr.vector3df(0, 0, 0),
    50,
    10, 50,
    40,
    512,
    chronoirr.SColorf(0.5, 0.5, 0.5)
)

# Add vehicle to visualization
application.Add(my_vehicle)

# Add terrain to visualization
application.Add(terrain_body)

# Set window title
application.GetDevice().setWindowCaption("PyChrono FEDA Vehicle Simulation - Irrlicht Visualization")

print("Irrlicht visualization initialized successfully.")

# ============================================================================
# CAMERA FOLLOW SYSTEM
# ============================================================================

class CameraFollower:
    """Camera system to follow the vehicle from a designated position"""
    
    def __init__(self, vehicle, offset):
        self.vehicle = vehicle
        self.offset = offset
        self.smoothing = 0.1  # Camera smoothing factor
        self.current_position = chrono.ChVector3d(0, 0, 0)
        
    def update(self, application):
        """Update camera position to follow vehicle"""
        # Get vehicle position and rotation
        vehicle_pos = self.vehicle.GetChassis().GetPos()
        vehicle_rot = self.vehicle.GetChassis().GetRot()
        
        # Calculate target camera position based on offset
        offset_world = vehicle_rot.Rotate(self.offset)
        target_position = vehicle_pos + offset_world
        
        # Smooth camera movement
        self.current_position = self.current_position + (target_position - self.current_position) * self.smoothing
        
        # Update camera
        camera = application.GetCamera()
        camera.setPosition(chronoirr.vector3df(
            self.current_position.x(),
            self.current_position.y(),
            self.current_position.z()
        ))
        
        # Look at vehicle
        look_target = chronoirr.vector3df(
            vehicle_pos.x(),
            vehicle_pos.y() + 1.0,  # Look slightly above vehicle
            vehicle_pos.z()
        )
        camera.setTarget(look_target)

# Create camera follower
camera_follower = CameraFollower(my_vehicle, CAMERA_OFFSET)
print(f"Camera configured to follow vehicle with offset: ({CAMERA_OFFSET.x()}, {CAMERA_OFFSET.y()}, {CAMERA_OFFSET.z()})")

# ============================================================================
# SIMULATION LOOP
# ============================================================================

print("\n" + "="*60)
print("Starting simulation loop...")
print(f"Target FPS: {SIMULATION_FPS}")
print(f"Duration: {SIMULATION_DURATION} seconds")
print("="*60 + "\n")

# Simulation variables
simulation_time = 0.0
frame_count = 0
step_size = 1.0 / SIMULATION_FPS

# Performance metrics
last_fps_update = 0
current_fps = 0
frame_count_fps = 0

# Application step
application.SetStepManage(chronoirr.E_PMS.NORMAL)
application.SetTimestep(step_size)
application.SetVideoframeRate(SIMULATION_FPS)

# Main simulation loop
while application.GetDevice().run():
    # Start frame
    application.BeginScene()
    
    # Clear scene
    application.GetVideoDriver().beginScene(True, True, chronoirr.SColor(255, 100, 100, 100))
    
    # Update driver inputs
    keyboard_driver.Synchronize(simulation_time)
    driver_inputs = keyboard_driver.GetInputs()
    
    # Update vehicle dynamics
    my_vehicle.Synchronize(
        simulation_time,
        driver_inputs,
        terrain_body
    )
    
    # Advance physics
    system.Update()
    
    # Update camera to follow vehicle
    camera_follower.update(application)
    
    # Render visualization
    application.DrawAll()
    
    # Draw additional info
    application.GetVideoDriver().endScene()
    
    # Update simulation time
    simulation_time += step_size
    frame_count += 1
    frame_count_fps += 1
    
    # Calculate and display FPS every second
    if simulation_time - last_fps_update >= 1.0:
        current_fps = frame_count_fps
        frame_count_fps = 0
        last_fps_update = simulation_time
        
        # Get vehicle information
        vehicle_speed = my_vehicle.GetVehicleSpeed()
        steering_angle = my_vehicle.GetSteering(0)
        
        # Display simulation info
        print(f"[{simulation_time:.1f}s] FPS: {current_fps} | "
              f"Speed: {vehicle_speed*3.6:.1f} km/h | "
              f"Steering: {steering_angle:.2f} rad")
    
    # Check for end of simulation
    if simulation_time >= SIMULATION_DURATION:
        print(f"\nSimulation completed: {simulation_time:.1f} seconds simulated")
        print(f"Total frames rendered: {frame_count}")
        break
    
    # Handle reset
    if application.GetDevice().getEventReceiver().IsKeyDown(chronoirr.EKEY_CODE.KEY_R):
        print("Resetting simulation...")
        # Reset vehicle position
        my_vehicle.GetChassis().SetPos(VEHICLE_INITIAL_POS)
        my_vehicle.GetChassis().SetRot(VEHICLE_INITIAL_ROT)
        my_vehicle.GetChassis().SetPos_dt(chrono.ChVector3d(0, 0, 0))
        my_vehicle.GetChassis().SetRot_dt(chrono.ChQuaterniond(1, 0, 0, 0))
        simulation_time = 0

print("\nSimulation ended. Closing application...")

# Cleanup
application.GetDevice().closeDevice()

print("Simulation script completed successfully.")