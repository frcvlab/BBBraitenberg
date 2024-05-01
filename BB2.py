#
# Example: Behavior Based extension of Braitenros
#

import BBbraitenrosT3 as br

v1 = br.Braitenros(simFlag=False)


#define a wander behavior
    
def B1(v1): # must include this syntax

    v1.vright = v1.random()   # use 'equal' rather than connect
    v1.vleft  = v1.random()   # random is predfined to return -0.5 to +0.5
    
    return br.Released # return whether the behavior is 'released' or not
    
def B2(v1):

    if (v1.rf_light+v1.lf_light)>1: # IRM: check if there is much light in view
    
        v1.vright=0.05*v1.lf_light  # contralateral 'connection' with weight
        v1.vleft =0.05*v1.rf_light
        
        return br.Released # say the behavior is released if you see light
        
    return not br.Released # and not otherwise
    
        
    
v1.addBehavior([B2,B1]) # add in the behaviors you defined to the vehicle
                        # in priority order between square parenthesis
                        # B2 is highest priority and B1 is only carried out if 
                        # B2 is 'not released'

v1.behave(showCamera=True) # shownCamera=True to see what vehicle sees


