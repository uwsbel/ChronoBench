import chrono
from chrono.irrlicht import *
from chrono import VehicleMap, ChSystemIrrlicht


chrono.ChSetChronoUseRealTime(True)
my_system = chrono.ChSystem()


my_system.Set_G_acc(chrono.ChVector(0, -9.81, 0))


ground = chrono.ChBodyEasyBox(100, 0.1, 100, 1000, chrono.ChMaterialSurfaceBase())
my_system.AddBody(ground)


viper_data = VehicleMap()
viper_data.LoadFromFile('viper_data.py')
viper = chrono.ChVehicleNodal(my_system, viper_data)
driver = chrono.ChVehicleDriver(viper)


device = chrono.ChIrrApp::GetDeviceFromCommandLine()
driver = chrono.ChIrrlichtDynamicsDriver(device)
driver.SetChSystem(my_system)


camera_pos = chrono.ChVectorF(0, 15, 20)
camera_target = chrono.ChVectorF(0, 0, 0)
driver.SetCamera(camera_pos, camera_target)


driver.SetAmbientLight(chrono.ChColorF(0.2f, 0.2f, 0.2f))
driver.SetDiffuseLight(chrono.ChColorF(0.8f, 0.8f, 0.8f))


TIME_STEPS = 1000
TIME_PERIOD = 10  
STEERING_CHANGE_TIME = 5  

for i in range(TIME_STEPS):
    
    if i < TIME_STEPS // 2 and i < TIME_PERIOD / STEERING_CHANGE_TIME:
        driver.SetSteer(viper, (i / (TIME_PERIOD / STEERING_CHANGE_TIME)) * 0.1)
    elif i >= TIME_STEPS // 2 and i < TIME_PERIOD / STEERING_CHANGE_TIME:
        driver.SetSteer(viper, -(i / (TIME_PERIOD / STEERING_CHANGE_TIME)) * 0.1)
    else:
        driver.SetSteer(viper, 0)

    
    my_system.DoStepDynamics(chrono.ChTime(0.01))
    driver.Render()


device.CloseDevice()