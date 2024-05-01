#
# Example: Behavior Based extension of Braitenros
#

import BBbraitenrosT3 as br

v1 = br.Braitenros(simFlag=False) # simFlag=True for Gazebo, omit for T3


#define a wander behavior
    
def wander(v1): # must include this syntax
    v1.vright = v1.random()   # use 'equal' rather than connect
    v1.vleft  = v1.random()   # random is predfined to return -0.5 to +0.5    
    return not br.Released # return whether the behavior is 'released' or not
 
def scared(v1):
    if (v1.rf_light+v1.lf_light)>10: # too much light
        v1.remember("scared",True)
    else:
        v1.remember("scared",False)
    return not br.Released
    
def runaway(v1):
    if v1.recall("scared"):
        v1.vright = 3*abs(v1.random())   # bigger intensity => kinesis
        v1.vleft  = 3*abs(v1.random())       
        return br.Released # return whether the behavior is 'released' or not
    return not br.Released
        
   
v1.addBehavior([scared,runaway,wander]) 

v1.addSkip(runaway,20) # stays alive for 5 units after being triggered

v1.behave(showCamera=True) # carry out the behavior


