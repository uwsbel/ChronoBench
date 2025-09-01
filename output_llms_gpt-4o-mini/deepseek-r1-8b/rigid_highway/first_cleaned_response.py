import pychrono as pychrono
import pychrono.vehicles as vehicles
import pychrono.input as input
import pychrono.render as render


pychrono.init()


scene = pychrono.Scene()
scene.set_renderer(render.Irrlicht())


ground = pychrono.objects.Plane()
ground.set_material(pychrono.materials.Material(
    color=[1, 1, 1],
    double_sided=False,
    texture="ground/ground_diffuse.tga",
    texture_normal="ground/ground_normal.tga"
))
scene.add_object(ground)


hmmwv = vehicles.HMMWV()
hmmwv.set_name("HMMWV")
hmmwv.set_contact_method(pychrono Contact)
hmmwv.set_tire_model("TMEASY")
hmmwv.set_position(pychrono.Vector3(
    x=0,
    y=0,
    z=0
))
hmmwv.set_orientation(pychrono.Vector3(
    x=1,
    y=0,
    z=0
))
scene.add_object(hmmwv)


terrain_col = pychrono.objects.Mesh()
terrain_col.load("Highway_col.obj")
terrain_vis = pychrono.objects.Mesh()
terrain_vis.load("Highway_vis.obj")
scene.add_object(terrain_col)
scene.add_object(terrain_vis)


driver = pychrono.input.Driver()
driver.set_max_steering_angle(1.0)
driver.set_max_throttle(1.0)
driver.set_max_braking_force(1.0)


hmmwv.set_damping(pychrono.Vector3(
    x=0.1,
    y=0.1,
    z=0.1
))


simulation_frequency = 50  
sim_loop = pychrono.SimLoop()
sim_loop.set_frequency(simulation_frequency)


def vehicle_update():
    
    steer = driver.get_steering()
    accel = driver.get_throttle()
    brake = driver.get_brake()
    
    
    if brake > 0.0:
        hmmwv.set_force(pychrono.Vector3(
            x=0,
            y=0,
            z=-brake*1000  
        ))
    if accel > 0.0:
        hmmwv.set_force(pychrono.Vector3(
            x=accel*1000,
            y=0,
            z=0
        ))
    
    
    if steer > 0.0:
        hmmwv.set_yaw(pychrono.YAW_LEFT)
    elif steer < 0.0:
        hmmwv.set_yaw(pychrono.YAW_RIGHT)


hmmwv.set_visible(True)
hmmwv.set_mesh("vehicle/HMMWV_mesh.obj")


sim_loop.start()
render.begin()
while True:
    
    vehicle_update()
    
    
    scene.update()
    render.draw()
    
    
    pychrono.sleep(1/simulation_frequency)