import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from math import factorial

# 1. Define Model Function
def exponential_model(x, a, b):
    return a * np.exp(b * x)

# 2. Data
x_data = np.array([1, 2, 3])
y_data = np.array([2.7, 7.4, 20.1])

# 3. Optimize (uses Levenberg-Marquardt usually)
params, covariance = curve_fit(exponential_model, x_data, y_data)
alpha, beta = params

print(f"Alpha: {alpha:.3f}")
print(f"Beta: {beta:.3f}")
# Output: Alpha: 0.987, Beta: 1.004


# 1. Configuration & Model Definition
def exponential_model(x, a, b):
    return a * np.exp(b * x)

# 2. Data Preparation
x_data = np.array([1, 2, 3])
y_data = np.array([2.7, 7.4, 20.1])
print(f"Using data points: x={x_data}, y={y_data}")

# 3. Core Algorithm: Curve Fitting (Non-linear Least Squares)
# Note: curve_fit does not need to manually build matrices like Normal Equation,
# it automatically solves iteratively internally
params, covariance = curve_fit(exponential_model, x_data, y_data)
alpha, beta = params

# --- Prepare auxiliary data for plotting ---

# A. Generate smooth curve data
# (Linear regression connects two points with a line,
# but exponential regression needs sufficiently dense points to show the curve)
x_smooth = np.linspace(min(x_data) - 0.5, max(x_data) + 0.5, 100)
y_smooth = exponential_model(x_smooth, alpha, beta)

# B. Calculate predicted values at original x points (specifically for drawing residual lines)
y_predict = exponential_model(x_data, alpha, beta)


# 4. Plot Visualization
plt.figure(figsize=(8, 6), dpi=120) # Set high DPI for PPT presentation

# 1. Scatter plot (Data Points)
plt.scatter(x_data, y_data, color='#4472C4', s=80, label='Observed Data', zorder=3)

# 2. Plot fitted curve (Fitted Curve)
plt.plot(x_smooth, y_smooth, color='#ED7D31', linewidth=3, label='Fitted Exponential Model', zorder=2)

# 3. Plot residual lines (Residuals) - Explaining least squares method
# Draw dashed lines from real points (y_data) to curve prediction points (y_predict)
for i in range(len(x_data)):
    plt.plot([x_data[i], x_data[i]], [y_data[i], y_predict[i]], 
             color='gray', linestyle='--', linewidth=1, alpha=0.6)

# 4. Chart decoration
# Dynamically generate title formula
plt.title(f'Exponential Least Squares Fitting\n$y = {alpha:.3f} \cdot e^{{{beta:.3f}x}}$', fontsize=16, pad=15)
plt.xlabel('x', fontsize=14)
plt.ylabel('y', fontsize=14)
plt.legend(fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)

# Set axis limits
plt.xlim(min(x_smooth), max(x_smooth))
# Leave some space at the bottom of Y-axis, or start from 0
plt.ylim(0, max(y_data) + 2) 

# Display
plt.tight_layout()
plt.show()

# Print results for verification
print(f"Fitting result: Alpha={alpha:.4f}, Beta={beta:.4f}")