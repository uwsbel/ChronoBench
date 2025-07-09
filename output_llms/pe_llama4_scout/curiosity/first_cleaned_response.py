import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as rob
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/chrono/data')


system = chrono.ChSystemSMC()


system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.RigidTerrain(system)
terrain_mat = chrono.ChContactMaterialSMC()
terrain_mat.SetFriction(0.9)
terrain_mat.SetRestitution(0.3)


patch = terrain.AddPatch(terrain_mat, 
                         chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                         10, 10)


terrain.Initialize()


rover = rob.CuriosityRover(system)


rover.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(chrono.CH_PI/2)))


rover.EnableCollision(True)


driver = rob.CuriosityRoverDriver(rover)


steering_input = chrono.ChFunction_Const(0.0)


speed_input = chrono.ChFunction_Const(0.5)


driver.Initialize(steering_input, speed_input)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()


vis.AddSkyBox()


vis.AddCamera(chrono.ChVector3d(5, 5, 5))


vis.AddTypicalLights()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))


while vis.Run():
    system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as rob
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    
    chrono.SetChronoDataPath('/path/to/chrono/data')
    system = chrono.ChSystemSMC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(system)
    terrain_mat = chrono.ChContactMaterialSMC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.3)
    patch = terrain.AddPatch(terrain_mat, 
                             chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                             10, 10)
    terrain.Initialize()

    
    rover = rob.CuriosityRover(system)
    rover.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleX(chrono.CH_PI/2)))
    rover.EnableCollision(True)

    
    driver = rob.CuriosityRoverDriver(rover)
    steering_input = chrono.ChFunction_Const(0.0)
    speed_input = chrono.ChFunction_Const(0.5)
    driver.Initialize(steering_input, speed_input)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Curiosity Rover Simulation')
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(5, 5, 5))
    vis.AddTypicalLights()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

    
    while vis.Run():
        system.DoStepDynamics(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == "__main__":
    main()