#this codeis for a wheel of a robot as a static gain system (PURE GAIN).
import control as ctrl
import matplotlib.pyplot as plt

G=5
H = 0.1

sys = ctrl.feedback(G, H)

# for a 10% stronger motor
sys2 = ctrl.feedback(1.1*G, H)

t1, y1 = ctrl.step_response(sys)
t2, y2 = ctrl.step_response(sys2)

print (f' this is he first transfer output {(sys)}')
print (f' this is he second transfer output {(sys2)}')
plt.plot(t1, y1, label="original")
plt.plot(t2, y2, label = "10% stronger motor")
plt.legend
plt.show()
