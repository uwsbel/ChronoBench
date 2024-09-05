import pychrono as chrono

import pychrono.visual

import pychrono.core

import pychrono.compiler




chrono.ChPrint("PyChrono epicyclic gears simulation\n")




my_chrono_instance = pychrono.ChSystem()




truss = chrono.ChBody()

truss.SetName('Fixed Truss')

truss.SetPos(chrono.ChVectorD(0, 0, 0))

truss.SetBodyFixed(True)

my_chrono_instance.Add(truss)




bar = chrono.ChBody()

bar.SetName('Rotating Bar')

bar.SetPos(chrono.ChVectorD(0, 1, 0))

bar.SetBodyFixed(False)

bar.AddRotationalDof(chrono.ChLinkWeldJoint())

bar.AddRotationalDof(chrono.ChLinkWeldJoint())

bar.AddRotationalDof(chrono.ChLinkWeldJoint())

my_chrono_instance.Add(bar)




gear1 = chrono.ChBody()

gear1.SetName('Gear 1')

gear1.SetPos(chrono.ChVectorD(1, 1, 0))

gear1.SetBodyFixed(False)

gear1.AddShape(chrono.ChBox(0.1, 0.1, 0.1))

my_chrono_instance.Add(gear1)


gear2 = chrono.ChBody()

gear2.SetName('Gear 2')

gear2.SetPos(chrono.ChVectorD(-1, 1, 0))

gear2.SetBodyFixed(False)

gear2.AddShape(chrono.ChBox(0.1, 0.1, 0.1))

my_chrono_instance.Add(gear2)


gear_motor = chrono.ChLinkWeldJoint()

gear_motor.Initialize(bar, gear1)

gear_motor.AddConstraint(chrono.ChLinkWeldJoint())

gear_motor.AddConstraint(chrono.ChLinkWeldJoint())

gear_motor.AddConstraint(chrono.ChLinkWeldJoint())




bar.SetRotationalDofVelocityLimit(100, chrono.ChPosDeg())

gear1.SetRotationalDofVelocityLimit(100, chrono.ChPosDeg())

gear2.SetRotationalDofVelocityLimit(100, chrono.ChPosDeg())




vis = pychrono.visual.ChVisualizer()

vis.SetWindowSize(1024, 768)

vis.SetWindowTitle('Epicyclic Gears Simulation')

vis.AddScene(my_chrono_instance)




material_brick = chrono.ChMaterialSurface('brick', 0.9, 0.05, 0.2, 0.2)

material_brick.SetColor(0.6, 0.6, 0.6)


material_gear = chrono.ChMaterialSurface('gear', 0.9, 0.05, 0.2, 0.2)

material_gear.SetColor(0.8, 0.8, 0.8)


vis.AddMaterial(material_brick)

vis.AddMaterial(material_gear)


vis.AddLight(chrono.ChLight())

vis.AddLight(chrono.ChLight())




compiler = pychrono.compiler.ChCompilator()

compiler.SetChronoFilesDirectory('chrono_data')

compiler.SetOutputDirectory('chrono_data')

compiler.Compile(my_chrono_instance, 'epicyclic_gears_simulation')




my_chrono_instance.DoStep(1, 1e-3)




vis.CloseWindow()