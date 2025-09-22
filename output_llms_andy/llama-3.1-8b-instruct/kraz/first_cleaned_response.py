import chrono as ch
import chrono.vehicle as chv
import chrono.vehicles as cv
import chrono.utils as cu
import chrono.irrlicht as ir


ch.init()


terrain = ch.RigidBodyDynamics()
terrain.SetGravity(ch.Vec3(0, 0, -9.81))


terrain.AddBody(ch.BoxBody(ch.Vec3(100, 100, 1), ch.Mass(0), ch.Vec3(0, 0, 0)))
terrain.AddForce(ch.Vec3(0, 0, 0), ch.Vec3(0, 0, 0), 1000)


kraz = chv.Kraz()
kraz.SetMass(2500)
kraz.SetInertiaTensor(ch.Mat33(1000, 0, 0, 0, 1000, 0, 0, 0, 1000))
kraz.SetPosition(ch.Vec3(0, 0, 0))
kraz.SetOrientation(ch.Quaternion(ch.Vec3(0, 0, 0), 1))
kraz.SetFriction(0.5)
kraz.SetRestitution(0.5)


terrain.AddBody(kraz)


driver = ch.Drivers()
driver.AddDriver(ch.Drivers.DriversType.CHROD, kraz)


ir.init()
camera = ir.Camera()
camera.SetPosition(ch.Vec3(0, 0, 10))
camera.SetTarget(ch.Vec3(0, 0, 0))
ir.addLight(ir.Light(ir.LIGHT_DIRECTIONAL, ch.Vec3(0, 0, -1), ch.Vec3(1, 1, 1)))


while True:
    
    terrain.Synchronize()
    terrain.Step()
    driver.Step()
    ir.Step()
    
    
    if not ir.isRunning():
        break


ch.finalize()