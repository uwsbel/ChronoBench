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
kraz.SetOrientation(ch.Q(ch.Vec3(0, 0, 0)))


driver = chv.Driver()
driver.SetVehicle(kraz)


ir.init()


ir.addCamera(ch.Vec3(0, 0, 5), ch.Vec3(0, 0, 0), ch.Vec3(0, 1, 0))
ir.addLight(ch.Vec3(10, 10, 10), ch.Color(1, 1, 1), 100)


def simulate():
    
    terrain.Simulate(0.01)
    driver.Simulate(0.01)
    ir.Simulate(0.01)
    kraz.Simulate(0.01)

    
    ir.beginScene()
    ir.draw(kraz)
    ir.endScene()


while True:
    simulate()
    ch.CheckAndPerformCollision(terrain)
    ch.CheckAndPerformContact(terrain)
    ir.pollEvents()