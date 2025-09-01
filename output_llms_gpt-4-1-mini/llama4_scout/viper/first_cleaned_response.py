import pychrono as chrono
import pychrono.irrlicht as irr
from pychrono.vehicle import *


chrono.ChronoEnvironment.set_G(chrono.ChVector3d(0, 0, -9.81))  


system = chrono.ChSystemNSC()


system.SetTimestep(0.01)
system.SetMaxIters(50)


ground = chrono.ChBody()
ground.SetKinematic(True)


ground_shape = chrono.ChCollisionShapeBox(ground, chrono.ChVector3d(100, 100, 0.1))
ground_shape.SetFriction(0.9)


system.Add(ground_shape)


viper = chrono.viper.Viper()


viper.Initialize(system)


viper.SetPos(chrono.ChVector3d(0, 0, 0.5))


system.Add(viper.GetChassisBody())


driver = chrono.ChVehicleDriver(viper)


driver.SetThrottle(0.5)
driver.SetSteering(0.0)


app = irr.IrrlichtApplication(system)


app.SetCamera(chrono.ChVector3d(0, -5, 1), chrono.ChVector3d(0, 0, 0.5))


app.SetLight(chrono.ChVector3d(-5, -5, 5), chrono.ChVector3d(1, 1, 1), True)


while app.Run():
    
    system.DoStepDynamics(0.01)

    
    t = system.GetChTime()
    steering = 0.5 * (1 - chrono.ChMath.cos(chrono.ChConst.Pi * t / 10))
    driver.SetSteering(steering)

    
    app.Render()

import pychrono as chrono
import pychrono.irrlicht as irr
from pychrono.vehicle import *

def main():
    
    chrono.ChronoEnvironment.set_G(chrono.ChVector3d(0, 0, -9.81))  

    
    system = chrono.ChSystemNSC()

    
    system.SetTimestep(0.01)
    system.SetMaxIters(50)

    
    ground = chrono.ChBody()
    ground.SetKinematic(True)

    
    ground_shape = chrono.ChCollisionShapeBox(ground, chrono.ChVector3d(100, 100, 0.1))
    ground_shape.SetFriction(0.9)

    
    system.Add(ground_shape)

    
    viper = chrono.viper.Viper()

    
    viper.Initialize(system)

    
    viper.SetPos(chrono.ChVector3d(0, 0, 0.5))

    
    system.Add(viper.GetChassisBody())

    
    driver = chrono.ChVehicleDriver(viper)

    
    driver.SetThrottle(0.5)
    driver.SetSteering(0.0)

    
    app = irr.IrrlichtApplication(system)

    
    app.SetCamera(chrono.ChVector3d(0, -5, 1), chrono.ChVector3d(0, 0, 0.5))

    
    app.SetLight(chrono.ChVector3d(-5, -5, 5), chrono.ChVector3d(1, 1, 1), True)

    
    while app.Run():
        
        system.DoStepDynamics(0.01)

        
        t = system.GetChTime()
        steering = 0.5 * (1 - chrono.ChMath.cos(chrono.ChConst.Pi * t / 10))
        driver.SetSteering(steering)

        
        app.Render()

if __name__ == "__main__":
    main()