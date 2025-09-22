import pychrono as pc
import pychrono.vehicles as vehicles
import pychronovisualization.irt as irt
import pychronoinput as input
import pychronorandom as random


pc.init()


sim = pc.Simulation()


vehicle = vehicles.Vehicle(sim, "ARTcar", pc.Vehicle.ContactMethod("wheels"), 
                        pc.Vehicle.VisualizationType("irt"), 
                        pc.Vehicle.DynamicsType("rigid"), 
                        pc.Vehicle.ContactType("none"))


vehicle.set_name("ARTcar")
vehicle.set_position(pc.Vec3(0, 0, 0))
vehicle.set_orientation(pc.Vec3(0, 0, 1))
vehicle.set_size(pc.Vec3(4, 2, 2))  
vehicle.set_mass(100)  


terrain = sim.createTerrain("rigid", "terrain", pc.TerrainType.Easy)
terrain.set_size(pc.Vec3(100, 100, 0))  
terrain.set_texture("terrain")  
terrain.set_position(pc.Vec3(0, 0, 0))
terrain.set_contact_method(pc.Terrain.ContactMethod("terrain"))


input_system = input.InputSystem(sim)
input_system.set_handler(input.Keyboard())
input_system.set_control_mapping({
    "steer_left": "A",
    "steer_right": "D",
    "brake": "F",
    "throttle": "W",
    "reverse": "S",
    "speed": "D"
})


renderer = irt.IrrRenderer()
renderer.set_lighting(pc.LightSetup(pc.LightType.DirectionalLight, 
                               pc.Vec3(0, 0, 100), 
                               pc.Vec3(0, 0, 100), 
                               0.5, 
                               0.5, 
                               0.5))
renderer.set_camera_position(pc.Vec3(10, 10, 10))
renderer.set.look_at(pc.Vec3(0, 0, 0))
renderer.set_up_vector(pc.Vec3(0, 1, 0))


sim.set_timestep(0.02)


camera = renderer.create_camera("perspective")
camera.set_fov(75)
camera.set_near(0.1)
camera.set_far(1000)


while True:
    
    input_system.update()
    
    
    steer_left = input_system.get_action("steer_left")
    steer_right = input_system.get_action("steer_right")
    brake = input_system.get_action("brake")
    throttle = input_system.get_action("throttle")
    
    
    vehicle.update()
    
    
    renderer.begin_frame()
    renderer.draw()
    renderer.end_frame()
    
    
    renderer.drawText(f"FPS: {int(sim.get_timestep() * 1000)}", 10, 50, 1, 1)
    
    
    if brake:
        vehicle.set_control(pc.Vehicle.Control(brake=1.0))
    else:
        vehicle.set_control(pc.Vehicle.Control())
        
    if steer_left or steer_right:
        vehicle.set_control(pc.Vehicle.Control(steer_left=steer_left, steer_right=steer_right))
    
    if throttle:
        vehicle.set_control(pc.Vehicle.Control(throttle=throttle))
    
    
    speed = 0
    if input_system.get_action("speed"):
        speed = 0.5
    elif input_system.get_action("reverse"):
        speed = -0.5
        
    vehicle.set_control(pc.Vehicle.Control(speed=speed))
    
    
    camera.set_position(pc.Vec3(10, 10, 10))
    camera.set_rotation(pc.Vec3(0, 0, 0))
    
    
    sim.update()
    
    
    pc.sleep(sim.get_timestep())