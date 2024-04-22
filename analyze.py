import numpy as np
import matplotlib.pyplot as plt

backLegSensorValues = np.load("data/backLegSensorValues.npy")
frontLegSensorValues = np.load("data/frontLegSensorValues.npy")
backLegTargetAngles = np.load("data/backLegTargetAngles.npy")
frontLegTargetAngles = np.load("data/frontLegTargetAngles.npy")

# plt.plot(backLegSensorValues, label="Back Leg", linewidth=3.5)
# plt.plot(frontLegSensorValues, label="Front Leg", linewidth=2)

plt.plot(backLegTargetAngles, label="Back Leg Target Angles", linewidth=2.5)
plt.plot(frontLegTargetAngles, label="Front Leg Target Angles")

plt.legend()
plt.show()
