#
# Example: Behavior Based extension of Braitenros
# 2 following vehicles, 1 leader
#

import BBbraitenrosT3 as br

v1 = br.Braitenros(simFlag=True) 
v2 = br.Braitenros(simFlag=True,name="blue") # will be the leader
v3 = br.Braitenros(simFlag=True)

    
 #too close    
def stop(v): 
    if (v.lf_touch>0 and v.lf_touched=="blue")or (v.rf_touch>0 and v.rf_touched=="blue"):
        v.vright = 0
        v.vleft  = 0
        return br.Released
    return not br.Released
    
def seefront(v): 
    if (v.lf_detect>0 ) or (v.rf_detect>0 ):
        v.vright = 0.25*v.lf_detect
        v.vleft= 0.25*v.rf_detect
        return br.Released
    return not br.Released

def backoff(v):
    if 0<v.lf_touch<0.4 or 0<v.rf_touch<0.4:
        v.vright,v.vleft=-0.15,-0.15
        return br.Released
    return not br.Released
    
def circle(v):
    v.vright= 0.2
    v.vleft = 0.2
    return br.Released

v3.tooClose=0.5
v3.not2Close=4.0
v3.addBehavior( [ backoff, stop,seefront] ) 

v1.tooClose=0.5
v1.not2Close=4.0
v1.addBehavior( [ backoff, stop,seefront] ) 

# the leader 
v2.addBehavior( circle )


br.multipleBehave( [v1,v2,v3] )

br.multiplePlotPosition( [v1,v2,v3] )


