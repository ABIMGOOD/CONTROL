# Closed-loop system is created using ct.feedback(G, H)
# The control library internally computes the negative feedback formula:
# T = G / (1 + G*H)
# So at this stage the open-loop system is converted into the closed-loop transfer function.
# ct.dcgain() does NOT form the feedback equation; it only evaluates the transfer function at s = 0
# to return the steady-state (DC) gain of the system.

import control as ct
G=ct.tf([50], [1]) 
H= ct.tf([0.1], [1]) 


sys = ct.feedback(G,H)

print (sys)
print ("DC gain = ", ct.dcgain(sys))