import pychron

import pychron.vehicle as vehicle

import pychron.terrain as terrain

import pychron.tire as tire

import pychron.camera as camera

import pychron.renderer as renderer

import pychron.materials as materials




chrono = pychron.Chrono()




car = vehicle.Car()

car.set_name("BMW E90 Sedan")

car.set_visualization(True)

car.set_collision(True)

car.set_tire_model(tire.TMEASY())




car.set_position([0, 0, 0])

car.set_orientation([0, 0, 0, 1])




terrain_obj = terrain.Terrain()

terrain_obj.set_rigid(True)

terrain_obj.set_texture("terrain_texture.jpg")

terrain_obj.set_logo("BMW_logo.jpg")




chrono.add(terrain_obj)




chrono.add(car)




camera_obj = camera.ChaseCamera()

camera_obj.set_position([10, 10, 10])

camera_obj.set_target(car)

camera_obj.set_look_at([0, 0, 0])




light = renderer.DirectionalLight()

light.set_direction([0, -1, 0])

chrono.add(light)




skybox = renderer.Skybox()

skybox.set_texture("skybox_texture.jpg")

chrono.add(skybox)




driver_system = pychron.DriverSystem()

driver_system.set_steering_control(True)

driver_system.set_throttle_control(True)

driver_system.set_brake_control(True)

car.set_driver_system(driver_system)




chrono.run()