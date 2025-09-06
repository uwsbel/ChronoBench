import chrono
import numpy as np
import math
import os
import pychrono
import pychrono.robot
import pychrono.irrlicht
import pychrono.vehicle
import sys

# 1. Initialize PyChrono Environment
try:
    chrono = chrono.ChSystem()
    chrono.SetChronoDataPath('/path/to/data')  # Replace with your data directory
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()

# 2. Define Physical Systems
try:
    mesh = chrono.ChMesh()
    mesh.SetAsRectangularSection(beam_wy, beam_wz)
    mesh.SetYoungModulus(0.01e9)
    mesh.SetShearModulus(0.01e9 * 0.3)
    mesh.SetRayleighDamping(0.000)
    hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
    mesh.AddNode(hnode1)
    mesh.AddNode(hnode2)
    belement1 = chrono.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(mesh)
    mesh.AddElement(belement1)

    # 3. Define Vehicle Model
    body_table = chrono.ChBody()
    body_table.SetPos(chrono.ChVector3d(0, -size_table_y / 2, 0))
    body_table.SetYoungModulus(0.01e9)
    body_table.SetShearModulus(0.01e9 * 0.3)
    body_table.SetRayleighDamping(0.000)
    hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
    body_table.AddNode(hnode1)
    body_table.AddNode(hnode2)
    belement1 = chrono.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(body_table)
    mesh.AddElement(belement1)

    # 4. Define Tire Model
    tire = chrono.ChTMEasy()
    tire.SetRadius(1.0)
    tire.SetTirePressure(0.8)
    tire.SetRollingResistance(0.5)
    tire.SetDamping(0.1)
    tire.SetCollisionType(chrono.ChCollisionSystem.Type_BULLET)
    tire.SetContactMethod(chrono.ChContactMethod.Type_TMEASY)
    tire.SetInitialPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.QuatFromAngleX(chrono.CH_PI))
    tire.Initialize()

    # 5. Define Driver Controls
    driver = chrono.ChLinkLockLock()
    driver.Initialize(body_table, body_floor, chrono.ChFramed())
    sys.Add(driver)

    # 6.  Visualization Setup
    vis = pychrono.irrlicht.VisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('MAN 10t Truck Simulation')
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

    # 7.  Run Simulation
    sys.DoStepDynamics(0.01)  # Adjust timestep as needed

    # 8.  Display the Simulation
    vis.Render()
    print("Simulation rendered.")

except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)