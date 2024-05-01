#
# Example: Behavior Based extension of Braitenros
#

import BBbraitenrosT3 as br

v1 = br.Braitenros(simFlag=False) # for the simulation


#define a wander behavior
    
def wander(v1): # must include this syntax

    v1.vright = v1.random()   # use 'equal' rather than connect
    v1.vleft  = v1.random()   # random is predfined to return -0.5 to +0.5
    
    return br.Released # return whether the behavior is 'released' or not
    
def phototaxis(v1):

    if (v1.rf_light+v1.lf_light)>1: # IRM: check if there is much light in view
    
        v1.vright=0.05*v1.lf_light  # contralateral 'connection' with weight
        v1.vleft =0.05*v1.rf_light
        
        return br.Released # say the behavior is released if you see light
        
    return not br.Released # and not otherwise
    
def bored(v1):
    
    v1.usingHabituated("bored",50,5)  # a 'memory state; get bored after 50 
                                      # and then not bored again after 5 more
    if v1.triggered("bored"): # IRM: release if "bored"
        v1.vright = 0.5 # just move the robot quickly in a straight line
        v1.vleft  = 0.5
        return br.Released   # release the behavior when bored  
              
    return not br.Released  # and not otherwise      
    
v1.addBehavior([bored,phototaxis,wander]) # add in the behaviors 
                        # in priority order between square parenthesis


v1.behave(showCamera=True) # carry out the behavior


