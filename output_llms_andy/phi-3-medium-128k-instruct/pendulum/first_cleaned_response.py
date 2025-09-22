from pychron.pychron import ChSystemNSC
from pychron.pychron_irrlicht_plugin import ChIrrApp
from pychron.pychron_irrlicht_plugin import ChIrrApp
from pychron.pychron_irrlicht_plugin import ChIrrVis
from pychron.pychron_irrlicht_plugin import ChIrrGeomSphere
from pychron.pychron_irrlicht_plugin import ChIrrLight
from pychron.pychron_irrlicht_plugin import ChIrrGeomBox
from pychron.pychron_irrlicht_plugin import ChIrrGeomRevoluteJoint
import pychron.pychron_irrlicht_plugin as irr
import pychron.pychron_irrlicht_plugin as irrlicht



chsystem = ChSystemNSC()



ground = ChIrrGeomBox(chsystem, name="Ground", pos=[0, 0, 0], size=[10, 1, 1], color=[0, 0, 0])



pendulum_body = ChIrrGeomSphere(chsystem, name="Pendulum", pos=[0, 0, 1], radius=1, mass=1, color=[1, 0, 0])



joint = ChIrrGeomRevoluteJoint(chsystem, name="PendulumJoint", bodyA=ground, bodyB=pendulum_body, axis=[0, 0, 0], pos=[0, 0, 0], axisP=pendulum_body.pos, axisQ=ground.pos)



pendulum_body.pos[1] = 2



chsystem.gravity[2] = -9.81



chsystem.set_time_step(0.01)



app = ChIrrApp(chsystem)
app.set_visualization(ChIrrVis(chsystem))
app.set_light(ChIrrLight(chsystem, name="Sun", color=[1, 1, 1], pos=[0, 10, 10], direction=[0, 0, -1])



app.start()



while app.is_running():

    pendulum_body.apply_impulse([0, -pendulum_body.mass * chsystem.gravity[2], 0, 0])

    chsystem.step()

    

    print(f"Position: {pendulum_body.pos}, Velocity: {pendulum_body.velocity}")

    app.render()