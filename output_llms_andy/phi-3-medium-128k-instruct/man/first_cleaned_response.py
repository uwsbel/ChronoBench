from pychron.core import *

from pychron.systems import *

from pychron.vehicle import Vehicle

from pychron.physics import *

from pychron.visualization import *

from pychron.vehicle.tire import TMEASY

from pychron.vehicle.driver import Driver

from pychron.physics.rigid import RigidBody

import pychron.visualization as vis

import pychron.collision as col




chrono = Chrono()




ground = RigidBody(name='Ground')

ground.add_box(pos=(0, 0, 0), size=(100, 100, 100))




vehicle = RigidBody(name='Truck')

vehicle.add_box(pos=(0, 0, 0), size=(8, 3, 3))




vehicle.add_collider(name='Chassis', size=(8, 3, 3))

vehicle.add_collider(name='Front', size=(2, 1, 1), pos=(4, 1.5, 0))

vehicle.add_collider(name='Rear', size=(2, 1, 1), pos=(-4, 1.5, 0))

vehicle.add_collider(name='Left', size=(1, 3, 1), pos=(0, 1.5, 0))

vehicle.add_collider(name='Right', size=(1, 3, 1), pos=(8, 1.5, 0))




vehicle.mass = 10000

vehicle.inertia = (1000, 1000, 1000)




vehicle.add_tire(name='FrontLeft', model=TMEASY, pos=(-4, 1.5, 0), size=(2, 1, 1))

vehicle.add_tire(name='FrontRight', model=TMEASY, pos=(4, 1.5, 0), size=(2, 1, 1))

vehicle.add_tire(name='RearLeft', model=TMEASY, pos=(-4, -1.5, 0), size=(2, 1, 1))

vehicle.add_tire(name='RearRight', model=TMEASY, pos=(4, -1.5, 0), size=(2, 1, 1))




driver = Driver(name='Driver')




driver.add_control('Steering', min_val=-0.5, max_val=0.5)

driver.add_control('Throttle', min_val=0, max_val=1)

driver.add_control('Brake', min_val=0, max_val=1)




vehicle.set_position((0, 0, 0))

vehicle.set_velocity((0, 0, 0))

vehicle.set_orientation((0, 0, 0))




system = System(name='TruckSystem')

system.add_body(vehicle)

system.add_driver(driver)




system.set_gravity((0, -9.81, 0))




vis = vis.IrrlichtVisualization(name='Visualization')

vis.set_camera(name='ChaseCamera', pos=(0, 10, 50))

vis.set_lighting(name='DirectionalLight', pos=(0, 10, 10), dir=(0, 0, -1))

vis.set_skybox('skybox.bmp')

vis.set_terrain_texture('terrain.bmp')

vis.set_logo('logo.bmp')




chrono.add_system(system)




ground.set_collision(True)

vehicle.set_collision(True)




chrono.run()