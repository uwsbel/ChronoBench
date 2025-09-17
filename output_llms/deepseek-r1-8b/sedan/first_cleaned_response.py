import pychrono as pychrono
import pychrono.render as pyrender
import pychrono.input as input
import pychronovehicles as veh


pychrono.init()
renderer = pyrender.Renderer()
renderer.setClearColor(0, 0, 0, 1)
renderer.setSize(1024, 768)


car = veh.createVehicle(veh.VehicleType.TEAMS_2010, "E90", pychrono.VehicleProperties(
    name="BMW E90",
    mass=1200,
    inertia=pychrono.VehicleInertia(
        xx=500,
        yy=400,
        zz=1000
    ),
    width=2.0,
    height=1.4,
    ground Clearance=0.5
))


terrain = pychrono.createRigidBody(pychrono.RigidBodyProperties(
    mass=0,
    inertia=pychrono.Inertia(0, 0, 0)
))
terrainShape = pychrono.Box(50, 50, 0.1)
terrain_body = pychrono.createRigidBody(pychrono.RigidBodyProperties(
    mass=0,
    inertia=pychrono.Inertia(0, 0, 0)
))
terrain_body.setShape(terrainShape)
terrain_body.setCollision(True)
terrain_body.setFriction(1.0)
terrain_body.setGround(True)


driver = pychrono.DriverSystem("Driver")
driver.addInputHandler(input.Keyboard())
driver.setVehicle(car)


renderer.addSkyBox("sky", "assets/skybox/ceiling1.tga", "assets/skybox/floor1.tga")
renderer.addDirectionalLight(0, 0, 1, 0.5, 0.5, 0.5)
renderer.setChaseCamera(car, 50, 50, 5)


car.getWheels()[0].setTireModel(veh.TireType.TMEASY)


car.setForce(0, 0, 0)
car.setTorque(0, 0, 0)
car.setAngularVelocity(0, 0, 0)


car.setPosition(pychrono.Vector3(0, 0.5, 0))
car.setRotation(pychrono.Vector3(0, 0, 0))


simulation = pychrono.Simulation(
    dt=0.001,
    maxSteps=10000,
    thread=0,
    gui=True
)


renderer.addVehicle(car, "BMW E90", "assets/vehicles/2010_BMW_E90.tga")
renderer.addTerrain(terrain_body, "Road", "assets/terrain/road_1.tga", 0, 0, 0, 50, 50)


while simulation.getStep() <= 0:
    
    inputs = input.getInputs()
    
    
    if inputs.keyboard[0].isPressed:
        car.setForce(0, 0, 100)
    if inputs.keyboard[1].isPressed:
        car.setForce(0, 0, -100)
    if inputs.keyboard[3].isPressed:
        car.setBrake(0, 0, 100)
    if inputs.keyboard[2].isPressed:
        car.setForce(0, 0, 0)
    
    
    simulation.updatePhysics()
    
    
    renderer.render()
    
    
    simulation.step()