print("Script started")
import control as ct
G=ct.tf([50], [1]) 
H= ct.tf([0.1], [1]) 


step = ct.feedback(G,H)

print (step)
print ("DC gain = ", ct.dcgain(step))