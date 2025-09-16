from pychrono import ChSystem, ChBody, ChCollisionShape, ChMaterialSurface, ChVectorD, ChQuaternionD, ChContact
from pychrono.irrlicht import IrrlichtVisualizer


system = ChSystem()


ground = ChBody(system)
ground.SetPos(ChVectorD(0, 0, 0))
ground_shape = ChCollisionShape()
ground_shape.SetBox(ChVectorD(10, 10, 1))  
ground.AddCollisionShape(ground_shape)
ground.SetBodyFixed(True)


rover = ChBody(system)
rover.SetPos(ChVectorD(0, 0, 1))
rover_shape = ChCollisionShape()
rover_shape.SetSphere(0.5)  
rover.AddCollisionShape(rover_shape)






visualizer = IrrlichtVisualizer(system)
visualizer.SetCamera(ChVectorD(5, 5, 5), ChVectorD(0, 0, 0))  
visualizer.SetAmbientLight(ChVectorD(0.5, 0.5, 0.5))
visualizer.SetDirectionalLight(ChVectorD(1, 1, 1), ChVectorD(0, 0, -1))
visualizer.SetShadowMapEnabled(True)






while True:
    system.DoStepDynamics(0.01)
    visualizer.Render()