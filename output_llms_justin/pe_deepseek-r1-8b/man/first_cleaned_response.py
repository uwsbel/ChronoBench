import os
import math
import numpy as np
import pychrono as chrono
from pychrono import robot, vehicle, irrrlicht, fea


sys = chrono.ChSystemSMC()


chrono.SetChronoDataPath(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from pychrono.vehicle import ARTcar


car = ARTcar(sys)
car.SetContactMethod(chrono.ChCollisionSystem.Type_NS_C)
car.SetChassisCollisionType(chrono.ChVehicleCollisionType.Rigid)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
car.Initialize()


sys.Add(car)


terrain = vehicle.RigidTerrain(car.GetSystem())
terrain.Initialize()


patch_material = chrono.ChMaterial()
patch_material.SetFriction(0.8)
patch_material.SetDampingF(0.2)
patch = terrain.AddPatch(
    patch_material,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    50,  // terrain length
    10   // terrain width
)
terrain.AddPatch(
    patch_material,
    chrono.ChCoordsysd(chrono.ChVector3d(100, 0, 0), chrono.QUNIT),
    50,
    10
)


vis = irrrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('MAN 10t Truck Simulation')
vis.AddLogo(chrono.GetChronoDataFile('pychrono-logo.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))  
vis.AddTypicalLights()


cam = vis.GetCamera()
cam.SetPosition(chrono.ChVector3d(0, 5, 10))
cam.SetRotation(chrono.QuatFromAxisEuler(chrono.CH_PI/2, 0, 0))  


num_tires = 4
tire_radius = 0.4
wheelbase = 2.5

for i in range(num_tires):
    tire = car.GetTires()[i]
    tire.SetTireModel(chrono.ChTireModel.TMEASY)
    tire.SetTireRadius(tire_radius)
    tire.SetWheelbase(wheelbase)
    tire.SetSuspensionStiffness(100000)
    tire.SetDampingF(0.1)
    tire.EnableCollision(True)


def control_function_throttle(time, speed):
    return speed  

def control_function_steering(time, speed, angle):
    return speed * np.clip(angle, -1, 1)  

def control_function_brake(time, speed):
    return -speed * 0.1  


car.SetControlFunctionThrottle(control_function_throttle)
car.SetControlFunctionSteering(control_function_steering)
car.SetControlFunctionBrake(control_function_brake)


sys.Add(car)
sys.Add(terrain)


sys.Initialize()


t = 0
dt = 0.01

while True:
    if vis.Run():
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        sys.DoStepDynamics(dt)
        t += dt
        
        
        vis.Update()
    
    
    car.UpdateControls(t, dt)
    
    
    if t > 0 and t % 1 == 0:
        print(f"Simulation Time: {t:.3f} seconds")