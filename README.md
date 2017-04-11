Passive-Dynamic-Walking
========
##### Simulation and animation of simple passive dynamic walking models in Matlab.
###### Version 1.1, 5-1-16
##### Download Repository: [ZIP Archive](https://github.com/horchler/Passive-Dynamic-Walking/archive/master.zip)

--------

[```simpwm```](https://github.com/horchler/Passive-Dynamic-Walking/blob/master/simpwm.m) simulates the [&#8220;Simplest Walking Model&#8221;](http://dx.doi.org/10.1115/1.2798313) passive dynamic walker for eight ```steps``` with a default slope, ```gam```, of 0.01 radians.  
  
[```fullwm```](https://github.com/horchler/Passive-Dynamic-Walking/blob/master/fullwm.m) simulates the full dynamics of a compass gait passive dynamic walker for eight ```steps``` with a default slope, ```gam```, of ```0.01``` radians and a ratio of foot-to-hip mass, ```B```, of ```0.01```.  
  
[```actuwm```](https://github.com/horchler/Passive-Dynamic-Walking/blob/master/actuwm.m) simulates the &#8220;Simplest Walking Model&#8221; passive dynamic walker [powered by toe-off impulse and a spring-like torque at hip](http://dx.doi.org/10.1115/1.1427703) for eight ```steps``` with a default slope, ```gam```, of ```0``` radians.  
  
[```wmview(y,gam,tci)```](https://github.com/horchler/Passive-Dynamic-Walking/blob/master/wmview.m) animates the passive dynamic data in ```y``` for slope angle ```gam``` and collision indices ```tci```.    
&nbsp;  

--------

References  
 1. M. Garcia, A. Chatterjee, A. Ruina, and M. Coleman, &#8220;The Simplest Walking Model: Stability, Complexity, and Scaling,&#8221; *ASME Journal of Biomedical Engineering*, Vol. 120, No. 2, pp. 281&ndash;288, 1998. [[http://dx.doi.org/10.1115/1.2798313](http://dx.doi.org/10.1115/1.2798313)]  
 2. M.W. Gomes &#8220;A Derivation of the Transisition Rule at Heelstrike which appears in the paper &#8216;The Simplest Walking Model: Stability, Complexity, and Scaling&#8217; by Garcia et al.,&#8221; *ONLINE*, pp. 1&ndash;3, Oct. 4, 1999. [[PDF](http://ruina.tam.cornell.edu/research/topics/locomotion_and_robotics/simplest_walking/simplest_walking_gomes.pdf)]  
 3. A.D. Kuo &#8220;Energetics of Actively Powered Locomotion Using the Simplest Walking Model,&#8221; *ASME Journal of Biomedical Engineering*, Vol. 124, No. 1, pp. 113&ndash;120, 2002. [[http://dx.doi.org/10.1115/1.1427703](http://dx.doi.org/10.1115/1.1427703)]  
&nbsp;  

--------

Andrew D. Horchler, *horchler @ gmail . com*, [biorobots.case.edu](http://biorobots.case.edu/)  
Created: 7-7-04, Revision: 1.1, 5-1-16  

This version tested with Matlab 9.0.0.341360 (R2016a)  
Mac OS X 10.11.4 (Build: 15E65), Java 1.7.0_75-b13  
Compatibility maintained back through Matlab 8.5 (R2015a)  
&nbsp;  

--------

Copyright &copy; 2004&ndash;2017, Andrew D. Horchler  
All rights reserved.  

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Case Western Reserve University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL ANDREW D. HORCHLER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.