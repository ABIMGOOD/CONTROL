import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

#Define the systems parameters

k_motor = 10  #Motor gain
tau = 0.5    #motor time constant
H=0.1      #Encoder feedback gain

t = np.linspace(0,5,1000)

def closed_loop_tf(Kc):
    #Returns the closed loop transfer function for a proportional controller gain Kc
    # Open loop = G(s) = kc*K_motor /() tau*s + 1)
    Output = [Kc * k_motor] #GAIN
    Input = [tau, 1]

    # Closed loop:Output / input + H*Output
    Input_closed =[tau, 1 + H * Kc *k_motor]

    return signal.TransferFunction(Output, Input_closed)

#Small Loop Gain Case

Kc_small = 5
Kc_small_new = 1.1 * Kc_small

sys_small = closed_loop_tf(Kc_small)
sys_small_new = closed_loop_tf(Kc_small_new)

t1, y1 = signal.step(sys_small, T=t)
t2, y2 = signal.step(sys_small_new, T=t)

plt.figure()
plt.plot(t1,y1)
plt.plot(t2,y2)
plt.title("small loop gain (kc = 5")
plt.xlabel("Time(Seconds")
plt.ylabel("wheel speed")
plt.show()

#Large loop Gain case

Kc_large = 50
Kc_large_new = 1.1 * Kc_large

sys_large = closed_loop_tf(Kc_large)
sys_large_new = closed_loop_tf(Kc_large_new)

t3, y3 = signal.step(sys_large, T=t)
t4, y4 = signal.step(sys_large_new, T=t)

plt.figure()
plt.plot(t3,y3)
plt.plot(t4,y4)
plt.title("Large Loop Gain(Kc = 50)")
plt.xlabel("Time(seconds)")
plt.ylabel("wheel speed")
plt.show()

print("Small Gain Final:", y1[-1])
print("Small Gain +10%:", y2[-1])
print("Large Gain Final:", y3[-1])
print("Large Gain +10%:", y4[-1])

