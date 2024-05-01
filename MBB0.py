#
# Example: Behavior Based extension of Braitenros
#

import BBbraitenrosT3 as br

v1 = br.Braitenros(simFlag=True) # simFlag=True for Gazebo, omit for T3
v2 = br.Braitenros(simFlag=True) # simFlag=True for Gazebo, omit for T3

#define a wander behavior
    
def B1(v1): # must include this syntax

    v1.vright = 10*v1.random()   # use 'equal' rather than connect
    v1.vleft  = 10*v1.random()   # random is predfined to return -0.5 to +0.5
    
    return br.Released # return whether the behavior is 'released' or not
    
v1.addBehavior(B1) # add in the behavior you defined to the vehicle
v2.addBehavior(B1) 

br.multipleBehave([v1,v2]) # takes a list of all the robots to behave

br.multiplePlotPosition([v1,v2]) # plats all positions on one graph


