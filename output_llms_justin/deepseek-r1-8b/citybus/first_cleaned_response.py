import PyChrono as pc
import Irrlicht as irt
import OpenGL
import math


pc.init()
irt.init()


scene = irt.Scene("Scene")


renderer = irt.createRenderer()
renderer.setClearColor(0, 0, 0, 1)
renderer.setSize(1024, 768)


terrain = pc.RigidTerrain()
terrain.setCustomTexture("terrain_1", "data/terrain_1.png")
terrain.create()
scene.addTerrain(terrain)


vehicle = pc.Vehicle()
vehicle.setMass(1500)  
vehicle.setTireModel(pc.TireModel.Soft)  
vehicle.setSuspensionModel(pc.SuspensionModel.Hard)  


city_bus_model = pc.loadModel("data/CityBus/CityBus.chp")
vehicle.setVehicleModel(city_bus_model)
vehicle.setVehicleName("CityBus")
vehicle.setInitialPosition(pc.Vector3(0, 0, 0))
vehicle.setInitialOrientation(pc.Vector3(1, 0, 0))  



body_mesh = pc.Mesh("data/CityBus/Body.chp")
body_mesh.setMaterial(pc.Material("diffuse", 1, 1, 1, 1))
vehicle.addPart(body_mesh, pc.Link2D(0, 1, 0, 0))


wheel_primitive = pc.Primitive("data/CityBus/Wheel.chp", 0, 0, 0, 1, 0)
wheel_primitive.setMaterial(pc.Material("diffuse", 1, 1, 1, 1))
wheel_primitive.setSize(pc.Vector3(0.5, 0.5, 0.5))
wheel_primitive.setPosition(pc.Vector3(-0.5, -0.5, 0))
wheel_primitive.setRotation(pc.Vector3(0, 0, 0))
wheel_primitive.add()
wheel_primitive.setInertia(pc.Vector3(0, 0, 0))  


for i in range(4):
    wheel = wheel_primitive.clone()
    wheel.setPosition(pc.Vector3(-0.5, 0.5, 0) if i % 2 == 0 else (0.5, 0.5, 0))
    wheel.setInertia(pc.Vector3(0, 0, 0))
    vehicle.addPart(wheel, pc.Link2D(0, 1, 0, 0))


camera = irt.Camera()
camera.setPosition(pc.Vector3(5, 5, 5))
camera.setLookAt(pc.Vector3(0, 0, 0))
camera.setUp(pc.Vector3(0, 1, 0))
renderer.setCamera(camera)


driver = pc.Driver()
driver.addControl(pc.SteeringControl(0.5, 0.5))  
driver.addControl(pc.ThrottleControl(0.5, 0.5))  
driver.addControl(pc.BrakeControl(0.5, 0.5))  
driver.setVehicle(vehicle)


light = irt.Light()
light.setPosition(pc.Vector3(0, 0, 10))
light.setDiffuseColor(1, 1, 1)
scene.addLight(light)


renderer.setClearColor(0.1, 0.1, 0.1, 1)
renderer.setLightingEnabled(True)


t = 0
while True:
    t += 1
    if t % 100 == 0:
        print(f"Time: {t}ms")
    
    
    vehicle.update()
    
    
    renderer.render(scene, camera)
    
    
    keys = pc.Keyboard()
    if keys.isKeyDown(pc.Keyboard.Key 'q'):
        break
    
    
    speed = 5
    if keys.isKeyDown(pc.Keyboard.Key 'w'):
        driver.setThrottle(0.5)
    elif keys.isKeyDown(pc.Keyboard.Key 's'):
        driver.setBrake(0.5)
    else:
        driver.setThrottle(0)
    
    if keys.isKeyDown(pc.Keyboard.Key 'a'):
        driver.setSteering(-0.5)
    elif keys.isKeyDown(pc.Keyboard.Key 'd'):
        driver.setSteering(0.5)
    else:
        driver.setSteering(0)
    
    
    torque = 500  
    vehicle.setEngineTorque(torque, 2)  
    
    
    renderer.drawToScreen()
    
    
    pc.sleep(1/50)


pc.shutdown()
irt.shutdown()