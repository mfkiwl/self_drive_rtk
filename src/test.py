import os
import numpy as np

p_a = np.array([1, -0.01])
p_b = np.array([1, 0])
len_a = np.linalg.norm(p_a)
len_b = np.linalg.norm(p_b)
dot_ba = p_b.dot(p_a)
yaw = np.arccos(dot_ba / (len_a * len_b))
k = p_b[0] * p_a[1] - p_b[1] * p_a[0]


# 000-090: k > 0, dot_ba > 0, 1 1
# 090-180: k > 0, dot_ba < 0, 1 0
# 180-270: k < 0, dot_ba < 0, 0 0
# 270-360: k < 0, dot_ba > 0, 0 1

print(k, dot_ba)
print(yaw)
if k < 0:
    yaw = 6.28 - yaw
print(yaw)
