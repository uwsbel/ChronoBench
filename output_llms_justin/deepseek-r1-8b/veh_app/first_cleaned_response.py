import pychrono as pychrono
from pychrono import Gator


simulation = pychrono.Simulation()
simulation.set_time_step(1/60.0)  


gator = Gator()
gator.set_name("Gator")
gator.set VisualizationType(gator.chassis, pychrono.VisualizationType.CYLINDER)
gator.set VisualizationType(gator.engine, pychrono.VisualizationType.SPHERE)
gator.set VisualizationType(gator.transmission, pychrono.VisualizationType.BOX)
gator.set VisualizationType(gator.suspension, pychrono.VisualizationType.CYLINDER)
gator.set VisualizationType(gator.wheels, pychrono.VisualizationType.CYLINDER)
gator.set VisualizationType(gator.cab, pychrono.VisualizationType.MESH)


driver = pychrono.Driver(gator, pychrono.DriverType.INTERACTIVE)
simulation.add_event_callback(driver.key_event_callback)


terrain = pychrono.RigidPlane()
terrain.set_name("Terrain")
terrain.set VisualizationType(pychrono.VisualizationType.TERRAIN)
simulation.add_object(terrain)


sensor_manager = pychrono.SensorManager()

light1 = pychrono.PointLight()
light1.set_name("Sensor Light 1")
sensor_manager.add_sensor_light(light1)
light2 = pychrono.PointLight()
light2.set_name("Sensor Light 2")
sensor_manager.add_sensor_light(light2)
light3 = pychrono.PointLight()
light3.set_name("Sensor Light 3")
sensor_manager.add_sensor_light(light3)


camera = pychrono.Camera()
camera.set_name("Simulation Camera")
camera.set_position([0, 0, 5])
simulation.add_object(camera)
sensor_manager.add_camera(camera, pychrono.SensorType.VISUALIZATION)


simulation.add_object(gator)
simulation.add_object(driver)
simulation.add_object(terrain)
simulation.add_object(sensor_manager)


while True:
    
    simulation.update()
    simulation.render()
    
    
    if driver.is_key_pressed(pychrono.keys.A):
        driver.set_speed( min(30, driver.get_speed() + 5) )
    if driver.is_key_pressed(pychrono.keys.S):
        driver.set_speed( max(-30, driver.get_speed() - 5) )
    if driver.is_key_pressed(pychrono.keys.D):
        driver.set_speed( min(30, driver.get_speed() + 5) )
    if driver.is_key_pressed(pychrono.keys.F):
        driver.set_speed( max(-30, driver.get_speed() - 5) )
    
    
    gator.update()
    terrain.update()
    sensor_manager.update()
    
    
    simulation.step()
    
    
    if not simulation.done():
        continue