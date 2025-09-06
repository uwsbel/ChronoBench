import chrono
import pychrono
import math
import numpy as np
import os
import sys
import time
import random
import irrlicht


chrono = chrono.ChSystemSMC()
chrono.ChSystemNSC()


my_system = chrono.ChBody()


my_system.SetPos(chrono.ChVector3d(0, 0, 0))
my_system.SetMass(100)
my_system.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
my_system.SetFixed(True)
my_system.SetCollisionModel.SetDefaultSuggestedEnvelope(0.01)


terrain = chrono.ChRigidTerrain()
terrain.SetContactMethod(chrono.ChContactMaterialNSC())
terrain.SetChassisCollisionType(chrono.ChCollisionSystem.Type_BULLET)
terrain.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
terrain.SetRigidTerrain(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
terrain.SetContactMethod(chrono.ChContactMaterialSMC())


mesh = chrono.ChMesh()
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0))))
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0))))
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_W, 0, 0))))
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, beam_W, 0))))
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, beam_L))))
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0))))
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, beam_L, 0))))
mesh.AddNode(chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0))))

mesh.AddElement(belement1)
mesh.AddElement(belement2)


driver = chrono.ChLinkLockLock()
driver.Initialize(mesh, my_system, chrono.ChFramed())
driver.SetDriver(my_system)


vis = irrlicht.Irrlicht.Scene()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('FEDA Simulation')
vis.AddCamera(chrono.ChVector3d(0, 0, 0))


sys.DoStepDynamics(0.01)
vis.Render()
vis.Pause(1)  
vis.Destroy()