from pychrono.core import *
from pychrono.visualization import *
from pychrono.irrlicht import *


chrono_init()


my_chrono_system = ChronoSystem()


my_chrono_system.addPhysicsSystem(MyPhysicsSystem())


floor = MyPhysicsSystem.createBox(0.5, 0.5, 1.0, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))
my_chrono_system.addPhysicsObject(floor)


crankshaft = MyPhysicsSystem.createCrankshaft(0.2, 0.1, 0.05, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
my_chrono_system.addPhysicsObject(crankshaft)


connecting_rod = MyPhysicsSystem.createRod(0.15, 0.1, 1.0, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
my_chrono_system.addPhysicsObject(connecting_rod)


piston = MyPhysicsSystem.createCylinder(0.05, 0.1, 0.5, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
my_chrono_system.addPhysicsObject(piston)


my_chrono_system.addJoint(MyPhysicsSystem.createSliderJoint(crankshaft, connecting_rod, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))
my_chrono_system.addJoint(MyPhysicsSystem.createCrankshaftJoint(crankshaft, piston, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))


motor = MyPhysicsSystem.createMotor(0.5, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))
my_chrono_system.addPhysicsObject(motor)


my_chrono_system.setMotorSpeed(motor, CH_MOTOR_TYPE_TORQUE, CH_MOTOR_LIMIT_NONE, CH_MOTOR_DIRECTION_CLOCKWISE, CH_MOTOR_SPEED_MIN, 100.0)


my_irrlicht_scene = MyIrrlichtScene()
my_irrlicht_scene.addLogo(MyIrrlichtScene.addLogo("path_to_logo.png"))
my_irrlicht_scene.addTexturedBox(MyIrrlichtScene.createBox(CH_VECTOR(0, 0, 0, CH_VECTOR(0.2, 0.2, 0.2), CH_VECTOR(0, 0, 0, 1, 1, 1, 0.5))


my_irrlicht_scene.setCameraPosition(CH_VECTOR(10, 10, 10), CH_VECTOR(0, 0, 0), CH_PI_OVER_TWO, CH_PI_OVER_TWO, CH_PI_OVER_TWO)
my_irrlicht_scene.setCameraViewUp(CH_VECTOR(0, 1, 0))


my_irrlicht_scene.addLight(MyIrrlichtScene.createDirectionalLight(CH_VECTOR(0, -1, -1), CH_VECTOR(1, 1, 1))


my_irrlicht_scene.addTexturedBox(MyIrrlichtScene.createBox(CH_VECTOR(0, 0, 0), CH_VECTOR(0.2, 0.2, 0.2), CH_VECTOR(0, 0, 0, 1, 1, 1, 0.5), "path_to_texture.jpg")


my_chrono_system.run(1.0)


my_irrlicht_scene.initViewPorts()
my_irrlicht_scene.initWindow(VI_RECT(640, 480), "Crank-Slider Simulation", VI_FULLSCREEN)
my_irrlicht_scene.addCustomLogo("path_to_logo.png")
my_irrlicht_scene.addWindowedRenderingWindow(True)
my_irrlicht_scene.run()


from pychrono.core import *
from pychrono.visualization import *
from pychrono.irrlicht import *
import logging


chrono_init()


my_chrono_system = ChronoSystem()


gear1 = MyPhysicsSystem.createGear(0.5, 0.2, 0.1, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
gear2 = MyPhysicsSystem.createGear(0.3, 0.2, 0.1, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
gear3 = MyPhysicsSystem.createGear(0.2, 0.2, 0.1, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
my_chrono_system.addPhysicsObject(gear1)
my_chrono_system.addPhysicsObject(gear2)
my_chrono_system.addPhysicsObject(gear3)


my_chrono_system.addJoint(MyPhysicsSystem.createGearJoint(gear1, gear2, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))
my_chrono_system.addJoint(MyPhysicsSystem.createGearJoint(gear2, gear3, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))


pendulum_length = 1.0
pendulum = MyPhysicsSystem.createPendulum(pendulum_length, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1), CH_VECTOR(0, 0, 0))
my_chrono_system.addPhysicsObject(pendulum)


magnet = MyPhysicsSystem.createMagnet(CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))
my_chrono_system.addPhysicsObject(magnet)


my_chrono_system.addDamping(pendulum, magnet, CH_DAMPING_TYPE_MAGNETIC, CH_DAMPING_MAGNETIC_STRENGTH, 0.1)


mass = MyPhysicsSystem.createRod(0.1, 0.1, 1.0, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
spring = MyPhysicsSystem.createSpring(0.1, 0.1, 1.0, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 1))
damper = MyPhysicsSystem.createDamper(0.1, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))
my_chrono_system.addPhysicsObject(mass)
my_chrono_system.addPhysicsObject(spring)
my_chrono_system.addPhysicsObject(damper)


my_chrono_system.addJoint(MyPhysicsSystem.createSpringDamperJoint(mass, spring, damper, CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0), CH_VECTOR(0, 0, 0))


my_irrlicht_scene = MyIrrlichtScene()
my_irrlicht_scene.addLogo(MyIrrlichtScene.addLogo("path_to_logo.png"))
my_irrlicht_scene.addTexturedBox(MyIrrlichtScene.createBox(CH_VECTOR(0, 0, 0), CH_VECTOR(0.2, 0.2, 0.2), CH_VECTOR(0, 0, 0, 1, 1, 1, 0.5))


my_irrlicht_scene.setCameraPosition(CH_VECTOR(10, 10, 10), CH_VECTOR(0, 0, 0), CH_PI_OVER_TWO, CH_PI_OVER_TWO, CH_PI_OVER_TWO)
my_irrlicht_scene.setCameraViewUp(CH_VECTOR(0, 1, 0))
my_irrlicht_scene.addLight(MyIrrlichtScene.createDirectionalLight(CH_VECTOR(0, -1, -1), CH_VECTOR(1, 1, 1))





logging.basicConfig(filename='simulation.log', level=logging.INFO)


try:
    while True:
        my_chrono_system.run(0.01)
        
        if my_chrono_system.getIteration() % 1000 == 0:
            my_irrlicht_scene.saveSnapshot("snapshot_%05d.png" % my_chrono_system.getIteration())
        
        logging.info(f"Iteration: {my_chrono_system.getIteration()}, Mass position: {mass.getPosition()}, Pendulum angle: {pendulum.getAngle()}, Spring force: {spring.getForce()}, Damping force: {damper.getForce()}")
except Exception as e:
    logging.error(f"Simulation error: {e}")


my_irrlicht_scene.initViewPorts()
my_irrlicht_scene.initWindow(VI_RECT(640, 480), "Complex Mechanical System Simulation", VI_FULLSCREEN)
my_irrlicht_scene.addCustomLogo("path_to_logo.png")
my_irrlicht_scene.addWindowedRenderingWindow(True)
my_irrlicht_scene.run()


from pychrono.irrlicht import *




def update_pendulum_length(value):
    pendulum_length = value
    pendulum.setLength(pendulum_length)


slider = MyIrrlichtScene.createSlider(0.5, 0.1, 1.0, 0.01, 1.0, update_pendulum_length)


my_irrlicht_scene.addSlider(slider)


slider.addEventHandler(MyIrrlichtScene.SliderEvent.OnValueChanged, update_pendulum_length)




try:
    while True:
        my_chrono_system.run(0.01)
        
        pendulum_length = slider.getValue()
        pendulum.setLength(pendulum_length)
        
        
except Exception as e:
    


logger = logging.getLogger("SimulationLogger")
logger.setLevel(logging.INFO)
console_handler = logging.StreamHandler()
file_handler = logging.FileHandler("simulation.log")
logger.addHandler(console_handler)
logger.addHandler(file_handler)


class IrrlichtHandler(logging.Handler):
    def __init__(self, scene):
        super().__init__()
        self.scene = scene
        self.text_element = scene.addTextElement()

    def emit(self, record):
        msg = self.format(record)
        self.scene.getTextElement().setText(msg)

irrlicht_handler = IrrlichtHandler(my_irrlicht_scene)
irrlicht_handler.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))
logger.addHandler(irrlicht_handler)




try:
    while True:
        my_chrono_system.run(0.01)
        logger.info(f"Iteration: {my_chrono_system.getIteration()}, Elapsed time: {time.time() - start_time}, Mass position: {mass.getPosition()}, Pendulum angle: {pendulum.getAngle()}, Spring force: {spring.getForce()}, Damping force: {damper.getForce()}")
        
        
except Exception as e:
    logger.error(f"Simulation error: {e}")


my_irrlicht_scene.initViewPorts()
my_irrlicht_scene.initWindow(VI_RECT(640, 480), "Complex Mechanical System Simulation", VI_FULLSCREEN)
my_irrlicht_scene.addCustomLogo("path_to_logo.png")
my_irrlicht_scene.addWindowedRenderingWindow(True)
my_irrlicht_scene.run()




my_chrono_system.setAdaptiveTimeStep(CH_MOTOR_SPEED_MIN, CH_MOTOR_SPEED_MAX, CH_MOTOR_DELTA, CH_MOTOR_STEP_TOLERANCE)




try:
    while True:
        my_chrono_system.run(0.01)
        
        if my_chrono_system.getIteration() % 1000 == 0:
            my_chrono_system.setAdaptiveTimeStep(CH_MOTOR_SPEED_MIN, CH_MOTOR_SPEED_MAX, CH_MOTOR_DELTA, CH_MOTOR_STEP_TOLERANCE)
        
        
except Exception as e:
    


from pychrono.irrlicht import *




def update_pendulum_length(value):
    pendulum_length = value
    pendulum.setLength(pendulum_length)


slider = MyIrrlichtScene.createSlider(0.5, 0.1, 1.0, 0.01, 1.0, update_pendulum_length)


slider.addEventHandler(MyIrrlichtScene.SliderEvent.OnValueChanged, update_pendulum_length)




try:
    while True:
        my_chrono_system.run(0.01)
        
        pendulum_length = slider.getValue()
        pendulum.setLength(pendulum_length)
        
        
except Exception as e:
    logger.error(f"Simulation error: {e}")


my_irrlicht_scene.initViewPorts()
my_irrlicht_scene.initWindow(VI_RECT(640, 480), "Complex Mechanical System Simulation", VI_FULLSCREEN)
my_irrlicht_scene.addCustomLogo("path_to_logo.png")
my_irrlicht_scene.addWindowedRenderingWindow(True)
my_irrlicht_scene.run()