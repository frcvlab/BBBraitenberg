#
# Example Braitenros program
#

import braitenrosT3 as br
v1 = br.Braitenros()

v1.connect(v1.vleft_connect, v1.rf_light_connect)
v1.connect(v1.vright_connect, v1.lf_light_connect)

v1.behave()

