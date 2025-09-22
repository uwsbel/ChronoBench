from pychrono.core import *
from pychrono.vehicle import *
from pychrono.vehicle_dynamics import *
from pychrono import visualization
from pychrono.irrlicht_utils import *


chrono_init()


uavbus = UAVBus()
uavbus.SetPos(Vec3(0, 0, 10))
uavbus.SetVehicleType(VEHICLE_TYPE_UAV)
uavbus.SetDamping(0.2)
uavbus.SetMass(1000)
uavbus.SetInertia(InertiaXXL(1000000, 1000000, 100000)
uavbus.SetFriction(0.5)
uavbus.SetRestitution(0.1)


terrain = Terrain()
terrain.SetPos(Vec3(0, 0, 0))
terrain.SetFriction(0.6)
terrain.SetRestitution(0.0)


driver = Driver()
driver.SetVehicle(uavbus)


irrlicht_application = create_default_application(True)
irrlicht_application.SetWindowSize(800, 600)
irrlicht_application.AddTypicalLights()
irrlicht_application.AddTypicalLens()
irrlicht_application.AddTypicalCamera()
irrlicht_application.AddTypicalViewport()


scene_graph = irrlicht_application.getSceneGraph()


scene_graph.AddElement(uavbus)
scene_graph.AddElement(terrain)


while irrlicht_application.GetApplication()->run():
    
    chrono_step(1.0/60.0, 10)

    
    irrlicht_application.SetSceneGraph(scene_graph)

    
    irrlicht_application.GetDevice()->SetViewport(irr.core.vector3df(0, 0, 0, 1, 1, 1))

    
    irrlicht_application.drawScene()

    
    irrlicht_application.GetDevice()->Step()