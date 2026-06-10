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






SIMULATION_FPS = 50
SIMULATION_DURATION = 100.0  
VISUALIZATION_STEP = 1.0 / SIMULATION_FPS


VEHICLE_INITIAL_POS = chrono.ChVector3d(0, 0.5, 0)
VEHICLE_INITIAL_ROT = chrono.ChQuatFromAngleZ(0)  
VEHICLE_MASS = 1500.0  


TERRAIN_LENGTH = 200.0  
TERRAIN_WIDTH = 100.0   
TERRAIN_WALL_HEIGHT = 1.0


CAMERA_OFFSET = chrono.ChVector3d(-8.0, 3.0, 0.0)  






system = chrono.ChSystemNSC()
system.SetNumThreads(4)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetMaxItersSolverSpeed(100)
system.SetMaxItersSolverStab(100)
system.Set timestep(0.002)


my_world = chrono.ChWorld()
system.Add(my_world)

print("PyChrono environment initialized successfully.")
print(f"Physics system created with {system.GetNumThreads()} threads.")






terrain_material = chrono.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.8)
terrain_material.SetRestitution(0.1)
terrain_material.SetPlasticCompliance(0.001)
terrain_material.SetYieldStress(1e7)


print("Creating rigid terrain...")


terrain_body = chrono.ChBody()
terrain_body.SetIdentifier(-1)
terrain_body.SetName("Terrain")
terrain_body.SetMass(0)  
terrain_body.SetPos(chrono.ChVector3d(0, -TERRAIN_WALL_HEIGHT, 0))
terrain_body.EnableCollision(True)
terrain_body.SetMaterialSurface(terrain_material)


terrain_shape = chrono.ChCollisionShapeBox(
    chrono.ChVector3d(TERRAIN_LENGTH / 2, TERRAIN_WALL_HEIGHT, TERRAIN_WIDTH / 2)
)
terrain_body.AddCollisionShape(terrain_shape, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
system.AddBody(terrain_body)


print("Creating terrain mesh with custom texture...")


def generate_terrain_mesh(length, width, resolution):
    
    vertices = []
    indices = []
    uvs = []
    
    step_x = length / resolution
    step_z = width / resolution
    
    
    for i in range(resolution + 1):
        for j in range(resolution + 1):
            x = -length / 2 + i * step_x
            z = -width / 2 + j * step_z
            vertices.append(chrono.ChVector3d(x, 0, z))
            uvs.append(chrono.ChVector2d(i / resolution * 10, j / resolution * 10))
    
    
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


texture_path = ""  
terrain_material_vis = chrono.ChVisualMaterial()
terrain_material_vis.SetAmbientColor(chrono.ChColor(0.4, 0.4, 0.4))
terrain_material_vis.SetDiffuseColor(chrono.ChColor(0.6, 0.6, 0.6))
terrain_material_vis.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))
terrain_material_vis.SetMetallic(0.0)
terrain_material_vis.SetRoughness(0.8)
terrain_mesh.material = terrain_material_vis

terrain_body.AddVisualShape(terrain_mesh, chrono.ChFrame(chrono.ChVector3d(0, -TERRAIN_WALL_HEIGHT, 0)))


print("Creating terrain boundaries...")

def create_terrain_wall(position, size, rotation_angle=0):
    
    wall = chrono.ChBody()
    wall.SetIdentifier(-1)
    wall.SetName(f"TerrainWall_{position}")
    wall.SetMass(0)  
    wall.EnableCollision(True)
    wall.SetMaterialSurface(terrain_material)
    
    wall.SetPos(position)
    wall.SetRot(chrono.ChQuatFromAngleY(rotation_angle))
    
    wall_shape = chrono.ChCollisionShapeBox(size)
    wall.AddCollisionShape(wall_shape, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
    
    
    wall_mesh = chrono.ChVisualShapeBox(size[0], size[1], size[2])
    wall.AddVisualShape(wall_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
    
    system.AddBody(wall)
    return wall


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





print("Initializing FEDA vehicle...")


vehicleAssembly = veh.ChVehicleAssembly()



feda_params = veh.FEDAParams()
feda_params.chassis_mass = VEHICLE_MASS
feda_params.chassis_dims = chrono.ChVector3d(4.0, 1.0, 2.0)
feda_params.wheelbase = 2.8
feda_params.front_suspension_height = 0.3
feda_params.rear_suspension_height = 0.3
feda_params.wheel_radius = 0.35
feda_params.wheel_width = 0.25
feda_params.max_steering_angle = 0.5  


my_vehicle = veh.FEDAVehicle(
    system,
    VEHICLE_INITIAL_POS,
    VEHICLE_INITIAL_ROT,
    feda_params
)

print(f"Vehicle created at position: ({VEHICLE_INITIAL_POS.x()}, {VEHICLE_INITIAL_POS.y()}, {VEHICLE_INITIAL_POS.z()})")
print(f"Vehicle mass: {VEHICLE_MASS} kg")





print("Configuring mesh visualization for all vehicle parts...")


chassis = my_vehicle.GetChassis()
chassis.SetVisualType(chrono.ChVisualizationType_MESH)


chassis_mesh = chrono.ChVisualShapeBox(
    feda_params.chassis_dims.x(),
    feda_params.chassis_dims.y(),
    feda_params.chassis_dims.z()
)
chassis.AddVisualShape(chassis_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0.5, 0)))


chassis_material = chrono.ChVisualMaterial()
chassis_material.SetDiffuseColor(chrono.ChColor(0.2, 0.4, 0.8))
chassis_material.SetSpecularColor(chrono.ChColor(0.3, 0.3, 0.3))
chassis_material.SetMetallic(0.6)
chassis_material.SetRoughness(0.3)
chassis_mesh.material = chassis_material


wheels = my_vehicle.GetWheels()
for i, wheel in enumerate(wheels):
    wheel.SetVisualType(chrono.ChVisualizationType_MESH)
    
    
    wheel_mesh = chrono.ChVisualShapeCylinder(
        feda_params.wheel_radius,
        feda_params.wheel_width
    )
    wheel.AddVisualShape(wheel_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0, 0)))
    
    
    wheel_material = chrono.ChVisualMaterial()
    wheel_material.SetDiffuseColor(chrono.ChColor(0.1, 0.1, 0.1))
    wheel_material.SetSpecularColor(chrono.ChColor(0.2, 0.2, 0.2))
    wheel_material.SetMetallic(0.8)
    wheel_material.SetRoughness(0.4)
    wheel_mesh.material = wheel_material


suspensions = my_vehicle.GetSuspensions()
for suspension in suspensions:
    suspension.SetVisualType(chrono.ChVisualizationType_MESH)
    
    
    suspension_mesh = chrono.ChVisualShapeBox(0.1, 0.3, 0.1)
    suspension.AddVisualShape(suspension_mesh, chrono.ChFrame(chrono.ChVector3d(0, 0.15, 0)))





print("Creating interactive driver system...")


driver_inputs = veh.DriverInputs()


keyboard_driver = veh.ChKeyboardDriver()


keyboard_driver.Initialize(system, my_vehicle)

print("Interactive driver system configured.")
print("Controls:")
print("  W/Up Arrow    - Accelerate (Throttle)")
print("  S/Down Arrow  - Brake")
print("  A/Left Arrow  - Steer Left")
print("  D/Right Arrow - Steer Right")
print("  Space         - Handbrake")
print("  R             - Reset vehicle")





print("Initializing Irrlicht visualization...")


application = chronoirr.ChIrrApp(
    system,
    "FEDA Vehicle Simulation",
    chronoirr.dimension2du(1280, 720),
    chronoirr.E_SHOW_TYPE.SHOW_ALL,
    chronoirr.E_DEVICE_TYPE.EIDT_DIRECT3D9
)


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


application.Add(my_vehicle)


application.Add(terrain_body)


application.GetDevice().setWindowCaption("PyChrono FEDA Vehicle Simulation - Irrlicht Visualization")

print("Irrlicht visualization initialized successfully.")





class CameraFollower:
    
    
    def __init__(self, vehicle, offset):
        self.vehicle = vehicle
        self.offset = offset
        self.smoothing = 0.1  
        self.current_position = chrono.ChVector3d(0, 0, 0)
        
    def update(self, application):
        
        
        vehicle_pos = self.vehicle.GetChassis().GetPos()
        vehicle_rot = self.vehicle.GetChassis().GetRot()
        
        
        offset_world = vehicle_rot.Rotate(self.offset)
        target_position = vehicle_pos + offset_world
        
        
        self.current_position = self.current_position + (target_position - self.current_position) * self.smoothing
        
        
        camera = application.GetCamera()
        camera.setPosition(chronoirr.vector3df(
            self.current_position.x(),
            self.current_position.y(),
            self.current_position.z()
        ))
        
        
        look_target = chronoirr.vector3df(
            vehicle_pos.x(),
            vehicle_pos.y() + 1.0,  
            vehicle_pos.z()
        )
        camera.setTarget(look_target)


camera_follower = CameraFollower(my_vehicle, CAMERA_OFFSET)
print(f"Camera configured to follow vehicle with offset: ({CAMERA_OFFSET.x()}, {CAMERA_OFFSET.y()}, {CAMERA_OFFSET.z()})")





print("\n" + "="*60)
print("Starting simulation loop...")
print(f"Target FPS: {SIMULATION_FPS}")
print(f"Duration: {SIMULATION_DURATION} seconds")
print("="*60 + "\n")


simulation_time = 0.0
frame_count = 0
step_size = 1.0 / SIMULATION_FPS


last_fps_update = 0
current_fps = 0
frame_count_fps = 0


application.SetStepManage(chronoirr.E_PMS.NORMAL)
application.SetTimestep(step_size)
application.SetVideoframeRate(SIMULATION_FPS)


while application.GetDevice().run():
    
    application.BeginScene()
    
    
    application.GetVideoDriver().beginScene(True, True, chronoirr.SColor(255, 100, 100, 100))
    
    
    keyboard_driver.Synchronize(simulation_time)
    driver_inputs = keyboard_driver.GetInputs()
    
    
    my_vehicle.Synchronize(
        simulation_time,
        driver_inputs,
        terrain_body
    )
    
    
    system.Update()
    
    
    camera_follower.update(application)
    
    
    application.DrawAll()
    
    
    application.GetVideoDriver().endScene()
    
    
    simulation_time += step_size
    frame_count += 1
    frame_count_fps += 1
    
    
    if simulation_time - last_fps_update >= 1.0:
        current_fps = frame_count_fps
        frame_count_fps = 0
        last_fps_update = simulation_time
        
        
        vehicle_speed = my_vehicle.GetVehicleSpeed()
        steering_angle = my_vehicle.GetSteering(0)
        
        
        print(f"[{simulation_time:.1f}s] FPS: {current_fps} | "
              f"Speed: {vehicle_speed*3.6:.1f} km/h | "
              f"Steering: {steering_angle:.2f} rad")
    
    
    if simulation_time >= SIMULATION_DURATION:
        print(f"\nSimulation completed: {simulation_time:.1f} seconds simulated")
        print(f"Total frames rendered: {frame_count}")
        break
    
    
    if application.GetDevice().getEventReceiver().IsKeyDown(chronoirr.EKEY_CODE.KEY_R):
        print("Resetting simulation...")
        
        my_vehicle.GetChassis().SetPos(VEHICLE_INITIAL_POS)
        my_vehicle.GetChassis().SetRot(VEHICLE_INITIAL_ROT)
        my_vehicle.GetChassis().SetPos_dt(chrono.ChVector3d(0, 0, 0))
        my_vehicle.GetChassis().SetRot_dt(chrono.ChQuaterniond(1, 0, 0, 0))
        simulation_time = 0

print("\nSimulation ended. Closing application...")


application.GetDevice().closeDevice()

print("Simulation script completed successfully.")