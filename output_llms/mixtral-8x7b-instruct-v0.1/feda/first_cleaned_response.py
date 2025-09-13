import pychrono.irrlicht as irr
import pychrono as chrono
import pychrono.vehicle as vehicle


chrono.Chrono.SetDefaultShadows Technique_SHADOWS_NONE
physics_engine = chrono.ChronoEngine()


driver = irr.IrrlichtDriver(physics_engine)
scene = driver.scene_manager
scene_node = scene.addEmptySceneNode()
scene_node.setPosition(chrono.ChVectorD(0, 0, 0))
scene.addLightSceneNode(scene_node, core.vector3df(30, 50, 0))


vehicle_model = vehicle.ChVehicleModelFEDA()
vehicle_model.SetChassisGeometry(chrono.ChBox(0.5, 0.2, 2.0))
vehicle_model.SetChassisMass(1500)
vehicle_model.SetSuspensionStiffness(50000)
vehicle_model.SetSuspensionDamping(2000)
vehicle_model.SetTireStiffness(200000)
vehicle_model.SetTireDamping(5000)
vehicle_model.SetTireFriction(1.2)
vehicle_model.SetContactMethod(vehicle.CONTACT_FEM)
vehicle_model.SetTireModel(vehicle.TIRE_MODEL_PACEJKA)


vehicle_system = physics_engine.AddVehicle(vehicle_model)
vehicle_system.SetPos(chrono.ChVectorD(0, 0, 0))
vehicle_system.SetChassisRot(chrono.ChQuaternionD(1, 0, 0, 0))


terrain_shape = chrono.ChHeightField()
terrain_shape.SetHeightMap(chrono.ChVectorD(100, 100))
terrain_shape.SetTexture(chrono.GetDataFile('textures/checkerboard.png'))
terrain_shape.SetMaterialSurface(chrono.ChMaterialSurface())
terrain_shape.SetMaterialSurface(chrono.ChMaterialSurface().SetFriction(0.5))
terrain = physics_engine.AddTerrain(terrain_shape)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))


camera_node = scene.addCameraSceneNode()
camera_node.setPosition(chrono.ChVector3f(0, 10, 10))
camera_node.setTarget(chrono.ChVector3f(0, 0, 0))
camera_node.setUpVector(chrono.ChVector3f(0, 1, 0))


camera_node.setFarValue(1000)
camera_node.setNearValue(0.1)
camera_node.setPosition(chrono.ChVector3f(0, 10, 10))
camera_node.setTarget(chrono.ChVector3f(0, 0, 0))
camera_node.setUpVector(chrono.ChVector3f(0, 1, 0))


vehicle_system.SetSteeringValue(0)
vehicle_system.SetThrottleValue(0)
vehicle_system.SetBrakeValue(0)


step = 1.0 / 50
while driver.run():
    physics_engine.DoStepDynamics(step)
    vehicle_system.Synchronize(physics_engine.GetChTime())
    camera_node.setTarget(vehicle_system.GetChassisBody().GetPos())
    driver.sleep(step)