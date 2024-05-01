#
# Example: Behavior Based extension of Braitenros
#

import BBbraitenrosT3 as br

v1 = br.Braitenros(simFlag=False) # simFlag=True for Gazebo, omit for T3


#define a wander behavior
    
def B1(v1): # must include this syntax

    v1.vright = v1.random()   # use 'equal' rather than connect
    v1.vleft  = v1.random()   # random is predfined to return -0.5 to +0.5
    
    return br.Released # return whether the behavior is 'released' or not
    
v1.addBehavior(B1) # add in the behavior you defined to the vehicle

v1.behave() # carry out the behavior


