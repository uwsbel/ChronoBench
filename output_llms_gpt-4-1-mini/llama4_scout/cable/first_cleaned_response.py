import pychrono as chrono
import pychrono.irrlicht as irr
import numpy as np


chrono.ChEngine.Initialize()


system = chrono.ChSystem()


system.Set_G_acc(chrono.ChVectorDbl(0, -9.81, 0))


app = irr.ChIrrApp(system, "ANCF Cable Beam Simulation", chrono.IrrlichtInfo())


app.SetTimestep(0.01)
app.SetRenderMode(irr.RenderMode.SolidWIREFRAME)


beam_length = 10.0  
beam_diameter = 0.1  
num_elements = 10
young_modulus = 200e9  
density = 7850  


cable_section = chrono.ChCableSectionA()
cable_section.SetYoungModulus(young_modulus)
cable_section.SetDensity(density)
cable_section.SetDiameter(beam_diameter)


cable_elements = []
for i in range(num_elements):
    cable_element = chrono.ChCableANCF()
    cable_element.SetCableSection(cable_section)
    cable_element.SetLength(beam_length / num_elements)
    cable_elements.append(cable_element)

    
    system.Add(cable_element)


for i in range(num_elements - 1):
    cable_elements[i].Connect(cable_elements[i + 1])


cable_elements[0].SetFixedFrame(chrono.ChFrameDbl(chrono.ChVectorDbl(0, 0, 0), chrono.ChQuaternionDbl(1, 0, 0, 0)))


for cable_element in cable_elements:
    visual_shape = chrono.ChVisualShapeCable()
    visual_shape.SetCableSection(cable_section)
    cable_element.AddVisualShape(visual_shape)


while app.GetDevice().run():
    
    system.DoStepDynamics(app.GetTimestep())

    
    app.Render()
    app.UpdateCamera()


app.GetDevice().close()

import pychrono as chrono
import pychrono.irrlicht as irr
import numpy as np

def main():
    
    chrono.ChEngine.Initialize()

    
    system = chrono.ChSystem()

    
    system.Set_G_acc(chrono.ChVectorDbl(0, -9.81, 0))

    
    app = irr.ChIrrApp(system, "ANCF Cable Beam Simulation", chrono.IrrlichtInfo())

    
    app.SetTimestep(0.01)
    app.SetRenderMode(irr.RenderMode.SolidWIREFRAME)

    
    beam_length = 10.0  
    beam_diameter = 0.1  
    num_elements = 10
    young_modulus = 200e9  
    density = 7850  

    
    cable_section = chrono.ChCableSectionA()
    cable_section.SetYoungModulus(young_modulus)
    cable_section.SetDensity(density)
    cable_section.SetDiameter(beam_diameter)

    
    cable_elements = []
    for i in range(num_elements):
        cable_element = chrono.ChCableANCF()
        cable_element.SetCableSection(cable_section)
        cable_element.SetLength(beam_length / num_elements)
        cable_elements.append(cable_element)

        
        system.Add(cable_element)

    
    for i in range(num_elements - 1):
        cable_elements[i].Connect(cable_elements[i + 1])

    
    cable_elements[0].SetFixedFrame(chrono.ChFrameDbl(chrono.ChVectorDbl(0, 0, 0), chrono.ChQuaternionDbl(1, 0, 0, 0)))

    
    for cable_element in cable_elements:
        visual_shape = chrono.ChVisualShapeCable()
        visual_shape.SetCableSection(cable_section)
        cable_element.AddVisualShape(visual_shape)

    
    while app.GetDevice().run():
        
        system.DoStepDynamics(app.GetTimestep())

        
        app.Render()
        app.UpdateCamera()

    
    app.GetDevice().close()

if __name__ == "__main__":
    main()