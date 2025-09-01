import pychrono

# Initialize the PyChrono environment
system = pychrono.ChSystemSMC()

# Add a ground object
ground = pychrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, False)
ground.SetPos(pychrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Add a box object
box = pychrono.ChBodyEasyBox(1, 1, 1, 1000, True, False)
box.SetPos(pychrono.ChVectorD(0, 1, 0))
system.Add(box)

# Set default parameters
system.Set_G_acc(pychrono.ChVectorD(0, -9.81, 0))
system.SetTimestepType(pychrono.ChTimestepper.Type_EULER_IMPLICIT)
system.SetTimestep(0.01)

# Simulate the system
system.DoStaticLinear()
system.DoDynamic()