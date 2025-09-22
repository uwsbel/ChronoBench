import chrono as chrono
import chronoIRR as ir


chrono.InitializeChWorld()
world = chrono.ChWorld()


terrain = world.AddRigidBody(chrono.ChBodyEasyBox(m=1000, 
                                                  pos=chrono.ChVectorD(0, 0, 0), 
                                                  color=chrono.ChColor(0.5, 0.5, 0.5), 
                                                  friction=0.8, 
                                                  restitution=0.2))
terrain.SetBodyFixed(True)


vehicle = world.AddRigidBody(chrono.ChBodyEasyBox(m=10000, 
                                                 pos=chrono.ChVectorD(5, 0, 0), 
                                                 color=chrono.ChColor(1, 0, 0), 
                                                 size=chrono.ChVectorD(2, 1, 1)))
vehicle.SetBodyFixed(False)
vehicle.AddForce(chrono.ChVectorD(0, -100, 0))


driver = world.AddForce(chrono.ChVectorD(0, 0, 0))
driver.SetTarget(vehicle.GetPos())


visual = ir.IrrlichtVisualizer(world, 'M113 Simulation')
visual.AddCamera(chrono.ChVectorD(0, 10, 10), chrono.ChVectorD(0, 0, 0))
visual.AddLight(chrono.ChVectorD(0, 10, 10), chrono.ChVectorD(0, 0, 0), 1000)


while visual.IsOpen():
    world.DoStepDynamics(0.01)
    visual.Update()


chrono.ClearChWorld()