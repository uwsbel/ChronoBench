import pychrono as chrono
import pychrono.irrlicht as ir
import numpy as np


physics = chrono.ChPhysicsEngine()
physics.Set_Gravity(chrono.ChVectorD(0, -9.81, 0))
integrator = chrono.ChIntegration_BasicEuler()
physics.SetIntegrator(integrator)
my_system = chrono.ChSystemNSC(physics)



terrain = chrono.ChTerrain()
terrain.SetTexture(chrono.ChTexture(chrono.ChFileTexture("path/to/terrain_texture.png")))
terrain.SetBBox(chrono.ChBoundBox(chrono.ChVector(-100, -1, -100), chrono.ChVector(100, 1, 100)))
my_system.Add(terrain)


bus = chrono.ChBodyEasyBox(0.5, 1.5, 2, 1000, True, True)
bus.SetPos(chrono.ChVector(0, 0.75, 0))
bus.SetRot(chrono.ChQuaternionD(chrono.ChMatrix33D(chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0))))
my_system.Add(bus)


tire = chrono.ChTire()
tire.SetBushingStiffness(chrono.ChVectorD(10000, 10000, 10000))
tire.SetBushingDamping(chrono.ChVectorD(100, 100, 100))
tire.SetFriction(0.8)
tire.SetRestLength(0.2)
tire.SetStiffness(100000)
bus.Add(tire)


steering = chrono.ChLinkLockRevolute()
steering.Initialize(bus, chrono.ChFrameD(chrono.ChVectorD(0, 0, -1), chrono.ChQuaternionD(chrono.ChMatrix33D(chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0)))))
steering.SetAngle(0)
my_system.Add(steering)

throttle = chrono.ChLinkLockPrismatic()
throttle.Initialize(bus, chrono.ChFrameD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(chrono.ChMatrix33D(chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0)))))
throttle.SetPos(0)
my_system.Add(throttle)

brake = chrono.ChLinkLockPrismatic()
brake.Initialize(bus, chrono.ChFrameD(chrono.ChVectorD(0, 0, -1), chrono.ChQuaternionD(chrono.ChMatrix33D(chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0)))))
brake.SetPos(0)
my_system.Add(brake)


asset = ir.ChIrrApp(my_system, physics, ir.CH_RTT_OPENGL)
asset.SetWindowSize(1024, 768)
asset.SetWindowTitle("CityBus Simulation")
asset.AddTypicalSky(irr.irrlicht.COLOR_SKY_BLUE1)
asset.AddTypicalLights()
asset.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))


bus_shape = chrono.ChBodyEasyMesh("path/to/bus_mesh.obj", 1000, True, True)
bus_shape.SetPos(chrono.ChVector(0, 0.75, 0))
bus_shape.SetRot(chrono.ChQuaternionD(chrono.ChMatrix33D(chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0))))
asset.Add(bus_shape)

wheel_visual = chrono.ChBodyEasyCylinder(0.2, 0.1, 1000, True, True)
wheel_visual.SetPos(chrono.ChVector(0, -0.75, 0.5))
wheel_visual.SetRot(chrono.ChQuaternionD(chrono.ChMatrix33D(chrono.ChVectorD(0, 0, 1), chrono.ChVectorD(1, 0, 0), chrono.ChVectorD(0, 1, 0))))
asset.Add(wheel_visual)


while asset.GetDevice().run():
    
    steering.SetAngle(np.sin(asset.GetTimer().GetTime() * 2))
    throttle.SetPos(np.sin(asset.GetTimer().GetTime() * 1.5))
    brake.SetPos(np.sin(asset.GetTimer().GetTime() * 2.5))

    my_system.DoStepDynamics(0.01)
    asset.DoStepRealTime(0.01)

    
    asset.BeginScene(True, True, ir.COLOR_WHITE)
    asset.DrawAll()
    asset.EndScene()