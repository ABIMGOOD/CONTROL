import control as ctrl
import numpy as np

H = 0.1

cases = [50, 5]

for G in cases:
    
    # original
    sys1 = ctrl.feedback(G, H)
    
    # 10% increase
    G_new = 1.1 * G
    sys2 = ctrl.feedback(G_new, H)
    
    print(f"\nForward gain G = {G}")
    print("Closed-loop gain:", ctrl.dcgain(sys1))
    
    print("After 10% increase in G")
    print("Closed-loop gain:", ctrl.dcgain(sys2))