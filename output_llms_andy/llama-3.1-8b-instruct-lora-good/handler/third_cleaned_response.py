import pychrono as ch
import chrono.ros as chros

import rpy.publisher from std.msg import Int

class MyCustom(chros.ChROS):
 

 def __init__(self, topic):
 super().__init__1)  

 self.topic = topic
 self.publisher rpy.publisher = None
 self.ticker = 0  
 def Initialize(self, interface: chros.ChROSPython) -> bool:
 
 print(f"Creating publisher for {self.topic...")
 Create publisher for specified topic self.publisher = interface.Get().create(Int, self.topic,1)
 return True
 def Tick(self, time):
 
 print(fPublish {self.ticker... msg = Int() self.ticker self.publisher
 self.t += 1
def main():
 
 sys = ch.ChSystemNS()
 sys.SetGravitational(ch.Ch3 0,9.81)  
 
 phys = ch.ChNSC()
 phys.Setiction 0.5  
 
 floor = chEasyBox(10, 1 100, True, True, phys)
 floor.SetPos(Ch3 0, 0)  
floor.Set(True)  
 floor.SetName base  
 sys.Add  
 Create box = EasyBox1 1 100 True, True, phys
box.Set(3 5,0) 
box.Set 0.2 ch 3 1,0) 
box.Set box  

 ros = chros.ChSP()

 register body ros.ChBody(25, box) box

 handler = ros.Add(floor, box)
 register custom = My(topic)

 ros.Initialize
 time 0
 time = 1e-3  
 end 30 
 realtime = ChRealStep() 
 while time  end:
 sysDynamics_step  
 time = sys.Get() 
 if ros(time, break) 
realtime.Spin  

if __name == "__main__":
 main()

import pychrono as ch
import chrono.ros as ch

import rpy from std.msg import Int


 class My(chros.ChROS):
 

 def __init__(self, topic):
 super().__init__1)  

 self = topic
 self rpy.publisher = None
 self.t =  
 def Initialize(self, interface: ch.ChSP) -> bool:
 
 print"Creating for {self...
 Create publisher for self = interface().create(Int, self,1)
 return def Tick(self):
 
 print Publish self.t msg = self.t self +=1
def main():
 
 sys = ChSystem()
 sys.SetGravit(ch3 0,9.81  
 
 = ChNSC()
.Set 0.5 
 floor = ChEasy(10,1, 100, True, True)
 floor.Set(Ch 0, 0) 
.Set(True) 
.SetName base 
sys.Add
 box =Easy(1,1 100 True, True)
.Set(5,0) box 
.Set 0.2 3,0) box.Set
Name 
 ros =Ch()
 register ros
 register body.Ch(25, box)
 transform.Ch(30)
 handler.Add(floor, box)
 register My(topic)
 ros
 ros.Initialize
 time 0
 = 1-3 
 end 30 duration
 = real
 while  end:
 sysDynamics  time = sys()
 if ros(time) break
 real.Spin  
if __ ==main__:
 main()