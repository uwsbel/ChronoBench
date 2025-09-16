import pychrono as chrono
import pychrono_sim as sim


sim_env = chrono.ChSystemNSC()


gator_vehicle = chrono.ChBodyEasyBox(1.5, 0.5, 2.0, 1000, 1000, 0.1)
gator_vehicle.SetPos(chrono.ChVectorD(0, 0, 0))
gator_vehicle.SetBodyFixed(True)
gator_vehicle.SetMass(10000.0)
gator_vehicle.SetInertiaXX(chrono.ChVectorD(2000.0, 2000.0, 2000.0))


engine = chrono.ChBodyEasyBox(0.1, 0.2, 0.3, 100, 100, 1000)
engine.SetPos(chrono.ChVectorD(0.25, 0.1, 0.3))
engine.SetBodyFixed(True)
engine.SetMass(5000.0)
engine.SetInertiaXX(chrono.ChVectorD(1000.0, 1000.0, 1000.0))

wheel = chrono.ChBodyEasyBox(0.1, 0.2, 0.3, 10, 10, 100)
wheel.SetPos(chrono.ChVectorD(0.4, 0.1, 0.3))
wheel.SetBodyFixed(True)
wheel.SetMass(100.0)
wheel.SetInertiaXX(chrono.ChVectorD(100.0, 100.0, 100.0))


terrain = chrono.ChBodyEasyBox(100, 100, 0.1, 0, 0, 0)
terrain.SetPos(chrono.ChVectorD(0, 0, -1))
terrain.SetBodyFixed(True)


sim_env.Add(gator_vehicle)
sim_env.Add(engine)
sim_env.Add(wheel)
sim_env.Add(terrain)


engine_vis = chrono.ChVisualizationType_Shape
wheel_vis = chrono.ChVisualizationType_Shape


engine_shape = chrono.ChBoxShape(chrono.ChVectorD(0.1, 0.2, 0.3))
wheel_shape = chrono.ChBoxShape(chrono.ChVectorD(0.1, 0.2, 0.3))

gator_vehicle.SetVisualization(engine_vis, engine_shape)
engine.SetVisualization(engine_vis, engine_shape)
wheel.SetVisualization(wheel_vis, wheel_shape)


sensor_manager = chrono.ChSensorManager()


point_light = chrono.ChLight()
point_light.SetAttenuation(chrono.ChVectorD(0.1, 0.1, 0.1))
point_light.SetPos(chrono.ChVectorD(10, 10, 10))
sensor_manager.Add(point_light)


camera = chrono.ChCamera()
camera.SetPosition(chrono.ChVectorD(0, 10, 10))
camera.SetFov(60)


sensor_manager.Add(camera)


sim_env.Add(sensor_manager)


driver_system = chrono.ChDriverSystem()


sim_env.Add(driver_system)


def run_simulation():
    while sim_env.DoStep(0.001, True) == 0:
        
        driver_system.Update()
        sim_env.DoStepDynamics(0.001, True)
        
        
        sensor_manager.Update()
        
        
        sensor_manager.Render()


run_simulation()