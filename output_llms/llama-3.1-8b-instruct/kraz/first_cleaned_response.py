import chrono as ch
import chronoIRR as irr


ch.InitChronic(0)


terrain = ch.RigidBody()
terrain.SetPos(ch.Vec3D(0, 0, 0))
terrain.SetMass(1000)
terrain.SetFriction(0.5)
terrain.SetRestitution(0.1)


vehicle = ch.RigidBody()
vehicle.SetPos(ch.Vec3D(10, 0, 0))
vehicle.SetMass(1500)
vehicle.SetInertiaTensor(ch.Mat33D(1000, 1000, 1000, 0, 0, 0))
vehicle.SetForce(ch.Vec3D(0, 0, 0))
vehicle.SetTorque(ch.Vec3D(0, 0, 0))


driver = ch.Driver()
driver.SetVehicle(vehicle)


visual = irr.Visual()
visual.SetCamera(ch.Vec3D(10, 10, 10), ch.Vec3D(0, 0, 0), ch.Vec3D(0, 1, 0))
visual.SetLighting(irr.LightingSettings())
visual.SetTerrain(terrain)


while True:
    
    ch.StepSimulation(0.01)
    visual.Update()
    driver.Update()
    if visual.GetFrameNumber() % 100 == 0:
        print(f"Frame {visual.GetFrameNumber()}: Vehicle velocity = {vehicle.GetVelocity().Length()} m/s")