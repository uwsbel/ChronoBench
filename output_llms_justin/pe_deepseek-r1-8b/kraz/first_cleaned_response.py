import os
import math
import numpy as np
from pychrono import chrono, contact, irrrlicht, fea, vehicle
from pychrono.contact import ChContactMaterialNSC, ChContactMaterialSMC
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere, ChVisualShapeCylinder
from pychrono.vehicle import Kraz, RigidTerrain, ARTcar


sys = chrono.ChSystemNSC()


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Kraz Vehicle Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()
vis.AddSkyBox()


kraz = Kraz(sys)
kraz.SetName("Kraz")
kraz.SetInitialPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
kraz.SetFixed(True)
sys.Add(kraz)


driver = kraz.GetDriver()
driver.SetName("Driver")
driver.SetFunction(chrono.ChFunction_Const(1.0))  
kraz.SetDriver(driver)


terrain = RigidTerrain(kraz.GetSystem())
terrain.SetName("Terrain")
terrain.Initialize()
terrain.SetFriction(0.5)
terrain.SetRestitution(0.2)


patch = terrain.AddPatch(
    material=ChContactMaterialNSC(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=10,
    width=10
)
terrain.Initialize()
sys.Add(terrain)


terrain_shape = ChVisualShapeBox(chrono.ChVector3d(50, 10, 5))
terrain.AddVisualShape(terrain_shape)
terrain_shape.SetColor(chrono.ChColor(1, 0.5, 0.5))  


kraz.AddVisualShape(ChVisualShapeBox(chrono.ChVector3d(2, 2, 2)))
kraz.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0))  

def simulation_step():
    global sys, vis, kraz, driver

    
    sys.DoStepDynamics(0.01)

    
    driver.SetSpeedFunction(chrono.ChFunction_Const(1.0))

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    contact_container = sys.GetContactContainer()
    if contact_container.GetNumContacts() > 0:
        for contact in contact_container.GetAllContacts():
            print(f"Contact detected between body {contact.GetBodyA().GetName()} and body {contact.GetBodyB().GetName()}")
            

simulation_step()

import os
import math
import numpy as np
from pychrono import chrono, contact, irrrlicht, fea, vehicle
from pychrono.contact import ChContactMaterialNSC, ChContactMaterialSMC
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere, ChVisualShapeCylinder
from pychrono.vehicle import Kraz, RigidTerrain, ARTcar

def main():
    
    sys = chrono.ChSystemNSC()

    
    chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

    
    vis = ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Kraz Vehicle Simulation")
    vis.Initialize()
    vis.AddCamera(chrono.ChVector3d(0, 5, 10))
    vis.AddTypicalLights()
    vis.AddSkyBox()

    
    kraz = Kraz(sys)
    kraz.SetName("Kraz")
    kraz.SetInitialPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
    kraz.SetFixed(True)
    sys.Add(kraz)

    
    driver = kraz.GetDriver()
    driver.SetName("Driver")
    driver.SetFunction(chrono.ChFunction_Const(1.0))  
    kraz.SetDriver(driver)

    
    terrain = RigidTerrain(kraz.GetSystem())
    terrain.SetName("Terrain")
    terrain.Initialize()
    terrain.SetFriction(0.5)
    terrain.SetRestitution(0.2)

    
    patch = terrain.AddPatch(
        material=ChContactMaterialNSC(),
        position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
        length=10,
        width=10
    )
    terrain.Initialize()

    
    terrain_shape = ChVisualShapeBox(chrono.ChVector3d(50, 10, 5))
    terrain.AddVisualShape(terrain_shape)
    terrain_shape.SetColor(chrono.ChColor(1, 0.5, 0.5))  

    
    kraz.AddVisualShape(ChVisualShapeBox(chrono.ChVector3d(2, 2, 2)))
    kraz.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0))  

    
    def simulation_step():
        nonlocal sys, vis, kraz, driver

        
        sys.DoStepDynamics(0.01)

        
        driver.SetSpeedFunction(chrono.ChFunction_Const(1.0))

        
        while vis.Run():
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        
        contact_container = sys.GetContactContainer()
        if contact_container.GetNumContacts() > 0:
            for contact in contact_container.GetAllContacts():
                print(f"Contact detected between body {contact.GetBodyA().GetName()} and body {contact.GetBodyB().GetName()}")
                

    simulation_step()

if __name__ == "__main__":
    main()