import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, veh, irrlight, fea
from pychrono.veh import RigidTerrain, RigidTire
from pychrono.robot import InteractiveDriver
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere
from pychrono.contact import ChContactMaterialSMC
from pychrono.systems import ChSystemSMC


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()
vis.AddSkyBox()


vehicle_width = 2.5  
vehicle_length = 4.0  
tire_radius = 0.5  
tire_hardness = 0.3  
soil_young_modulus = 1.0e6  
soil_shear_modulus = 0.2 * soil_young_modulus  
soil_friction = 0.6  
deformable_terrain_height = 1.0  


init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.QuatFromAngleX(chrono.CH_PI / 2)  


car = veh.ARTcar(sys)
car.SetContactMethod(veh.ContactMethod_SMC)
car.SetChassisCollisionType(veh.ChassisCollisionType_Rigid)
car.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
car.Initialize()


sys.Add(car)


terrain = RigidTerrain(sys)
terrain.SetGroundType(terrain.GroundType_Deformable)
terrain.SetYoungModulus(soil_young_modulus)
terrain.SetShearModulus(soil_shear_modulus)
terrain.SetFriction(soil_friction)
terrain.Initialize()


patch = terrain.AddPatch(
    patch_material=chrono.ChContactMaterialSMC(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=vehicle_length,
    width=vehicle_width,
    height=deformable_terrain_height
)
terrain.patch = patch  


def update_patch_position():
    patch_pos = car.GetBody().GetPos()
    patch.SetPos(chrono.ChCoordsysd(patch_pos, init_rot))

sys.GetContactContainer().RegisterAddContactCallback(update_patch_position)


driver = InteractiveDriver(sys, 0, 0, 0)  
driver.EnableThrottle(True)
driver.EnableSteering(True)
driver.EnableBrake(True)


driver.SetControlMapping(
    throttle_axis=1,
    steer_axis=0,
    brake_axis=-1
)



chassis_shape = ChVisualShapeBox(chrono.ChVector3d(vehicle_length, vehicle_width, 0.2))
chassis_shape.SetColor(chrono.ChColor(1, 0.5, 0))
car.AddVisualShape(chassis_shape)


wheel_radius = tire_radius + 0.1  
for i in range(4):
    wheel_pos = car.GetBody().GetPos() + chrono.ChVector3d(
        (vehicle_width / 2 - 1) * (1 if i % 2 == 0 else -1),
        0,
        0
    )
    wheel_shape = ChVisualShapeSphere(wheel_radius)
    wheel_shape.SetColor(chrono.ChColor(0, 0.5, 1))  
    car.AddVisualShape(wheel_shape)


sys.SetTimeStep(0.01)


vis.SetFrameLimit(50)  
vis.EnableVSync(True)


while vis.Run():
    
    update_patch_position()
    
    
    sys.DoStepDynamics(sys.GetTimeStep())
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, veh, irrlight, fea
from pychrono.veh import RigidTerrain, RigidTire
from pychrono.robot import InteractiveDriver
from pychrono.irrlicht import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere
from pychrono.contact import ChContactMaterialSMC
from pychrono.systems import ChSystemSMC


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()
vis.AddSkyBox()


vehicle_width = 2.5  
vehicle_length = 4.0  
tire_radius = 0.5  
tire_hardness = 0.3  
soil_young_modulus = 1.0e6  
soil_shear_modulus = 0.2 * soil_young_modulus  
soil_friction = 0.6  
deformable_terrain_height = 1.0  


init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.QuatFromAngleX(chrono.CH_PI / 2)  


car = veh.ARTcar(sys)
car.SetContactMethod(veh.ContactMethod_SMC)
car.SetChassisCollisionType(veh.ChassisCollisionType_Rigid)
car.SetInitPosition(chrono.ChCoordsysd(init_pos, init_rot))
car.Initialize()


sys.Add(car)


terrain = RigidTerrain(sys)
terrain.SetGroundType(terrain.GroundType_Deformable)
terrain.SetYoungModulus(soil_young_modulus)
terrain.SetShearModulus(soil_shear_modulus)
terrain.SetFriction(soil_friction)
terrain.Initialize()


patch = terrain.AddPatch(
    patch_material=chrono.ChContactMaterialSMC(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=vehicle_length,
    width=vehicle_width,
    height=deformable_terrain_height
)
terrain.patch = patch  


def update_patch_position():
    patch_pos = car.GetBody().GetPos()
    patch.SetPos(chrono.ChCoordsysd(patch_pos, init_rot))

sys.GetContactContainer().RegisterAddContactCallback(update_patch_position)


driver = InteractiveDriver(sys, 0, 0, 0)  
driver.EnableThrottle(True)
driver.EnableSteering(True)
driver.EnableBrake(True)


driver.SetControlMapping(
    throttle_axis=1,
    steer_axis=0,
    brake_axis=-1
)



chassis_shape = ChVisualShapeBox(chrono.ChVector3d(vehicle_length, vehicle_width, 0.2))
chassis_shape.SetColor(chrono.ChColor(1, 0.5, 0))
car.AddVisualShape(chassis_shape)


wheel_radius = tire_radius + 0.1  
for i in range(4):
    wheel_pos = car.GetBody().GetPos() + chrono.ChVector3d(
        (vehicle_width / 2 - 1) * (1 if i % 2 == 0 else -1),
        0,
        0
    )
    wheel_shape = ChVisualShapeSphere(wheel_radius)
    wheel_shape.SetColor(chrono.ChColor(0, 0.5, 1))  
    car.AddVisualShape(wheel_shape)


sys.SetTimeStep(0.01)


vis.SetFrameLimit(50)  
vis.EnableVSync(True)


while vis.Run():
    
    update_patch_position()
    
    
    sys.DoStepDynamics(sys.GetTimeStep())
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()