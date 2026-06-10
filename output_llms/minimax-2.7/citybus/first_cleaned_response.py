import os
import math
import numpy as np


import pychrono.core as chrono
import pychrono.vehicle as chronovehicle
import pychrono.irrlicht as irrlicht






chrono.SetChronoDataPath("C:/Chrono/data/")


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
my_system.SetSolverType(chrono.ChSolverSolverType.SOR)
my_system.SetMaxItersSolverSpeed(50)
my_system.SetMaxItersSolverStab(50)
my_system.SetStepSize(0.001)


simulation_fps = 50
step_size = 1.0 / simulation_fps

print("=" * 60)
print("PyChrono CityBus Simulation")
print("=" * 60)





print("\n[INFO] Creating rigid terrain...")


terrain_height = 0.0  
terrain_length = 200.0  
terrain_width = 100.0   


ground_material = chrono.ChMaterialSurfaceNSC()
ground_material.SetFriction(0.9)
ground_material.SetRestitution(0.1)
ground_material.SetCompliance(0.0)
ground_material.SetComplianceT(0.0)


ground = chrono.ChBody()
ground.SetName("ground")
ground.SetPos(chrono.ChVectorD(0, terrain_height, 0))
ground.SetBodyFixed(True)
ground.SetMaterialSurface(ground_material)


ground_collision = chrono.ChCollisionShapeBox(ground, 
                                               terrain_length / 2, 
                                               0.1, 
                                               terrain_width / 2)
ground.AddCollisionShape(ground_collision, chrono.ChFrameD())
ground.EnableCollision(True)

my_system.Add(ground)


vis_material = chrono.ChVisualizationMaterial()
vis_material.SetAmbientColor(chrono.ChColor(0.4, 0.4, 0.4))
vis_material.SetDiffuseColor(chrono.ChColor(0.5, 0.5, 0.5))
vis_material.SetSpecularColor(chrono.ChColor(0.1, 0.1, 0.1))

ground_vis = chrono.ChVisualizationShapeBox(terrain_length, 0.1, terrain_width)
ground_vis.SetMaterial(vis_material)
ground.AddVisualizationShape(ground_vis)

print(f"[INFO] Terrain created: {terrain_length}m x {terrain_width}m")





print("\n[INFO] Creating CityBus vehicle...")


vehicle_location = chrono.ChVectorD(0, 1.0, 0)
vehicle_orientation = chrono.Q_ROT_Y -  


citybus = chronovehicle.CityBus_VehicleSimple(my_system, 
                                               chrono.ChCoordsysD(vehicle_location, vehicle_orientation))


tire_model = chronovehicle.TireModelType.RIGID
citybus.SetTireModel(tire_model)


citybus.Initialize()


print("[INFO] Setting visualization types...")


chassis_vis = citybus.GetChassis().GetVisualizationShape(0)
if chassis_vis:
    chassis_vis.SetMesh("citybus_body.obj", chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"))
    print("[INFO] Chassis: Using mesh visualization")


front_suspension = citybus.GetVehicle().GetSuspension(0)
rear_suspension = citybus.GetVehicle().GetSuspension(1)


for i in range(4):
    wheel = citybus.GetWheel(i).GetSpindle()
    wheel_vis = chrono.ChVisualizationShapeCylinder(0.4, 0.3)
    wheel_vis.SetMaterial(vis_material)
    wheel.AddVisualizationShape(wheel_vis)
    print(f"[INFO] Wheel {i}: Using primitive (cylinder) visualization")





print("\n[INFO] Creating interactive driver system...")


driver = chronovehicle.ChDriver(citybus.GetVehicle())


driver.SetMaxSteering(0.5)  
driver.SetMaxThrottle(1.0)   
driver.SetMaxBraking(1.0)   


steering_input = 0.0
throttle_input = 0.0
brake_input = 0.0

print("[INFO] Driver system initialized with controls:")
print("       - Steering: Left/Right Arrow Keys")
print("       - Throttle: Up Arrow Key")
print("       - Brake: Down Arrow Key")
print("       - Reset: R Key")
print("       - Quit: Q Key")





print("\n[INFO] Setting up Irrlicht visualization...")


myapplication = irrlicht.ChIrrApp(
    my_system,
    "CityBus Simulation",
    irrlicht.dimension2du(1280, 720)
)


myapplication.AddTypicalSky()
myapplication.AddTypicalLights(irrlicht.dimension2df(0.5, 0.5), 
                                 irrlicht.dimension2df(0.5, 0.5),
                                 100, 80)
myapplication.AddTypicalCamera(irrlicht.vector3df(10, 5, -10),
                                 irrlicht.vector3df(0, 2, 0))


myapplication.Add(myapplication.CreateGrid(20, 50, chrono.ChColor(0.3, 0.3, 0.3), True))


myapplication.Add(citybus)


camera_distance = 15.0  
camera_height = 5.0    
camera_offset = chrono.ChVectorD(0, camera_height, -camera_distance)

print("[INFO] Visualization system initialized")





def update_camera_follow(application, vehicle, offset):
    
    vehicle_pos = citybus.GetChassis().GetPos()
    camera_target = irrlicht.vector3df(vehicle_pos.x, vehicle_pos.y + 2, vehicle_pos.z)
    camera_pos = irrlicht.vector3df(
        vehicle_pos.x + offset.x,
        vehicle_pos.y + offset.y,
        vehicle_pos.z + offset.z
    )
    application.GetDevice().getSceneManager().getActiveCamera().setTarget(camera_target)
    application.GetDevice().getSceneManager().getActiveCamera().setPosition(camera_pos)

def get_vehicle_speed(vehicle):
    
    vel = vehicle.GetChassis().GetPos_dt()
    return math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

def format_time(seconds):
    
    minutes = int(seconds // 60)
    secs = seconds % 60
    return f"{minutes:02d}:{secs:05.2f}"





print("\n" + "=" * 60)
print("STARTING SIMULATION")
print("=" * 60)
print(f"Target FPS: {simulation_fps}")
print(f"Step size: {step_size:.4f} seconds")
print("=" * 60)


running = True
simulation_time = 0.0
frame_count = 0


key_states = {
    'left': False,
    'right': False,
    'up': False,
    'down': False
}


class ChIrrEventReceiver(irrlicht.ChIrrEventReceiver):
    def __init__(self, key_states):
        super().__init__()
        self.key_states = key_states
        
    def OnEvent(self, event):
        if event.EventType == irrlicht.EET_KEY_INPUT_EVENT:
            key = event.KeyInput.Key
            pressed = event.KeyInput.PressedDown
            
            if key == irrlicht.KEY_LEFT:
                self.key_states['left'] = pressed
            elif key == irrlicht.KEY_RIGHT:
                self.key_states['right'] = pressed
            elif key == irrlicht.KEY_UP:
                self.key_states['up'] = pressed
            elif key == irrlicht.KEY_DOWN:
                self.key_states['down'] = pressed
            elif key == irrlicht.KEY_KEY_Q:
                if not pressed:
                    global running
                    running = False
            elif key == irrlicht.KEY_KEY_R:
                if not pressed:
                    
                    citybus.GetChassis().SetPos(vehicle_location)
                    citybus.GetChassis().SetRot(vehicle_orientation)
                    citybus.GetChassis().SetPos_dt(chrono.ChVectorD(0, 0, 0))
                    citybus.GetChassis().SetRot_dt(chrono.ChVectorD(0, 0, 0))
                    
        return False


event_receiver = ChIrrEventReceiver(key_states)
myapplication.GetDevice().setEventReceiver(event_receiver)


while running and myapplication.GetDevice().run():
    
    myapplication.BeginScene()
    myapplication.DrawAll()
    
    
    if key_states['left']:
        steering_input = min(steering_input + 0.02, driver.GetMaxSteering())
    elif key_states['right']:
        steering_input = max(steering_input - 0.02, -driver.GetMaxSteering())
    else:
        steering_input *= 0.95  
    
    throttle_input = 1.0 if key_states['up'] else 0.0
    brake_input = 1.0 if key_states['down'] else 0.0
    
    
    driver.SetSteering(steering_input)
    driver.SetThrottle(throttle_input)
    driver.SetBraking(brake_input)
    
    
    citybus.Synchronize(simulation_time, 
                        chronovehicle.DriverInputs(steering_input, throttle_input, brake_input),
                        my_system)
    
    
    my_system.DoStepDynamics(step_size)
    simulation_time += step_size
    frame_count += 1
    
    
    update_camera_follow(myapplication, citybus, camera_offset)
    
    
    if frame_count % 50 == 0:
        speed_ms = get_vehicle_speed(citybus)
        speed_kmh = speed_ms * 3.6
        print(f"Time: {format_time(simulation_time)} | "
              f"Speed: {speed_kmh:.1f} km/h | "
              f"Steering: {steering_input:.2f} | "
              f"Throttle: {throttle_input:.2f} | "
              f"Brake: {brake_input:.2f}")
    
    
    myapplication.EndScene()
    
    
    if not myapplication.GetDevice().run():
        running = False


print("\n" + "=" * 60)
print("SIMULATION COMPLETED")
print("=" * 60)
print(f"Total simulation time: {format_time(simulation_time)}")
print(f"Total frames: {frame_count}")
print(f"Average FPS: {frame_count / simulation_time:.2f}")
print("=" * 60)


myapplication.GetDevice().closeDevice()