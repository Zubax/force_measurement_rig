import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# Simulate incoming data
x = np.linspace(0, 10, 1000)
freq = 500

x1 = x[:300]
x2 = x[300:500]
x3 = x[500:]

y1 = np.sin(freq * x1)
y2 = np.sin(freq * x2) + (x2 - x2[0]) * 10
y3 = np.sin(freq * x3)
y = np.concatenate([y1, y2, y3])

# --- Peak detection logic ---
window_size = 20
value_window = deque(maxlen=window_size)
derivative_window = deque(maxlen=window_size)
peak_detected = False
peak_index = None

for i in range(len(y)):
    value_window.append(y[i])

    if len(value_window) >= 20:
        dy = value_window[0] - value_window[-1]
        derivative_window.append(dy)
        print(f"{i}: {dy}")

    # Detect peak: if previous derivatives are mostly positive, and current one is strongly negative
    if len(derivative_window) == window_size:
        # Check last 3 derivatives
        last_ders = list(derivative_window)[-3:]
        if (last_ders[0] > 2 and last_ders[1] > 2 and last_ders[2] < 2 and not peak_detected):
            peak_detected = True
            peak_index = i
            print(f"Peak detected at index {i}, x = {x[i]:.3f}, y = {y[i]:.3f}")
            break

# --- Plot the full signal and mark the peak ---
plt.plot(x, y, label='Signal')
if peak_detected:
    plt.axvline(x[peak_index], color='r', linestyle='--', label='Detected Peak')
    plt.plot(x[peak_index], y[peak_index], 'ro')
plt.legend()
plt.title("Peak Detection Based on First Derivative")
plt.xlabel("x")
plt.ylabel("y")
plt.grid(True)
plt.show()
