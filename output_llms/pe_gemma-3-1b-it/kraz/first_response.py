import chrono
import numpy as np
import math
import os
import sys
import time

# Initialize PyChrono environment
try:
    chrono = chrono.ChSystem()
    print("PyChrono environment initialized successfully.")
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()

# --- System Initialization ---
try:
    # Set Chrono Data Path
    chrono.SetChronoDataPath('/path/to/data')
    print("Chrono data path set successfully.")
except Exception as e:
    print(f"Error setting Chrono data path: {e}")
    exit()

# --- Contact Settings ---
try:
    # Set Contact Materials
    material_nsc = chrono.ChContactMaterialNSC()
    print("Material Nsc initialized.")
except Exception as e:
    print(f"Error initializing Contact Material Nsc: {e}")
    exit()

try:
    material_smc = chrono.ChContactMaterialSMC()
    print("Material SMC initialized.")
except Exception as e:
    print(f"Error initializing Material SMC: {e}")
    exit()

try:
    # Set Collision Model
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
    print("Default Suggested Envelope set.")
except Exception as e:
    print(f"Error setting Default Suggested Envelope: {e}")
    exit()

# --- Visualization Settings ---
try:
    # Set Visualization System
    vis = chrono.ChVisualSystemIrrlicht()
    print("Visualization System initialized.")
except Exception as e:
    print(f"Error initializing Visualization System: {e}")
    exit()

try:
    # Set Window Size
    vis.SetWindowSize(1024, 768)
    print("Window Size set.")
except Exception as e:
    print(f"Error setting Window Size: {e}")
    exit()

try:
    # Set Window Title
    vis.SetWindowTitle('Simulation Window')
    print("Window Title set.")
except Exception as e:
    print(f"Error setting Window Title: {e}")
    exit()

try:
    # Initialize Visualization
    vis.Initialize()
    print("Visualization initialized.")
except Exception as e:
    print(f"Error initializing Visualization: {e}")
    exit()


# --- Body Initialization ---
try:
    # Create Body
    body = chrono.ChBody()
    print("Body initialized.")
except Exception as e:
    print(f"Error initializing Body: {e}")
    exit()

# --- Joint and Link Settings ---
try:
    # Revolute Joint
    rev_joint = chrono.ChLinkLockRevolute()
    print("Revolute Joint initialized.")
except Exception as e:
    print(f"Error initializing Revolute Joint: {e}")
    exit()

# --- Spherical Joint Settings ---
try:
    pris_joint = chrono.ChLinkLockSpherical()
    print("Spherical Joint initialized.")
except Exception as e:
    print(f"Error initializing Spherical Joint: {e}")
    exit()

# --- Motor Joint Settings ---
try:
    # Motor Joint
    motor = chrono.ChLinkMotorRotationSpeed()
    print("Motor Joint initialized.")
except Exception as e:
    print(f"Error initializing Motor Joint: {e}")
    exit()

# --- Simulation Loop ---
try:
    # Run Simulation
    sys.DoStepDynamics(0.01)
    print("Simulation loop started.")
    while True:
        # Update Simulation
        sys.DoStepDynamics(0.01)

        # Render Visualization
        vis.Render()
        print("Visualization rendered.")
        time.sleep(0.1)  # Short delay for visualization

except Exception as e:
    print(f"Error during simulation loop: {e}")
    sys.Exit()
    
# --- Example Simulation (Beam) ---
try:
    # Beam Simulation
    beam = fea.ChBeamSectionEulerAdvanced()
    beam.SetAsRectangularSection(beam_wy, beam_wz)
    beam.SetYoungModulus(0.01e9)
    beam.SetShearModulus(0.01e9 * 0.3)
    beam.SetRayleighDamping(0.000)
    hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
    beam.AddNode(hnode1)
    beam.AddNode(hnode2)
    belement1 = fea.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(beam)
    beam.AddElement(belement1)

    # Set Initial Position
    beam.SetPos(chrono.ChVector3d(0, 0, 0))

    # Set Rotation
    beam.SetRotation(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT, 0.0))

    # Create Link
    link_shaker = chrono.ChLinkLockLock()
    link_shaker.Initialize(beam, beam_floor, chrono.ChFramed())

    # Set Motion
    beam.SetMotionY(chrono.ChFunctionSine(0.001, 1.5))

    # Add the Link
    link_shaker.AddLink(beam, beam_floor)

    # Run the simulation
    sys.Add(link_shaker)

    print("Beam simulation completed.")

except Exception as e:
    print(f"Error during beam simulation: {e}")
    sys.Exit()