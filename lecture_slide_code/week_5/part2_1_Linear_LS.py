import numpy as np
import matplotlib.pyplot as plt

# 1. Prepare Data
X = np.array([[1, 1], [1, 2], [1, 3]]) # Design Matrix
y = np.array([2, 3, 5])                # Target Vector

# 2. Compute Normal Equation
# beta = (X^T * X)^-1 * X^T * y
Xt = X.T
XtX = np.dot(Xt, X)
XtX_inv = np.linalg.inv(XtX)
Xty = np.dot(Xt, y)

beta = np.dot(XtX_inv, Xty)

print(f"Intercept: {beta[0]:.2f}")
print(f"Slope: {beta[1]:.2f}")
# Output: Intercept: 0.33, Slope: 1.50



# 1. Data Configuration
USE_ORIGINAL_DATA = True  

# How many points to generate if not using original data?
NUM_POINTS = 20  

# 2. Data Preparation
if USE_ORIGINAL_DATA:
    x = np.array([1, 2, 3])
    y = np.array([2, 3, 5])
    print("Using original PPT data (3 points)...")

# 3. Core Algorithm: Normal Equation (same as PPT)
# Build Design Matrix X (add a column of 1s)
X_b = np.c_[np.ones((len(x), 1)), x]  # shape: (N, 2)

# Normal equation solution: beta = (X^T * X)^-1 * X^T * y
theta_best = np.linalg.inv(X_b.T.dot(X_b)).dot(X_b.T).dot(y)

intercept = theta_best[0]
slope = theta_best[1]

# Calculate predicted values
y_predict = X_b.dot(theta_best)

# 4. Plot Visualization
plt.figure(figsize=(8, 6), dpi=120) # Set high DPI for PPT presentation

# 1. Scatter plot (Data Points)
plt.scatter(x, y, color='#4472C4', s=80, label='Observed Data', zorder=3)

# 2. Plot fitted line (Regression Line)
plt.plot(x, y_predict, color='#ED7D31', linewidth=3, label='Fitted Model', zorder=2)

# 3. Plot residual lines (Residuals) - Key to explaining least squares!
# These are the vertical distances from points to the line
for i in range(len(x)):
    plt.plot([x[i], x[i]], [y[i], y_predict[i]], 
             color='gray', linestyle='--', linewidth=1, alpha=0.6)

# 4. Chart decoration
plt.title(f'Linear Least Squares Fitting\n$y = {intercept:.2f} + {slope:.2f}x$', fontsize=16, pad=15)
plt.xlabel('x', fontsize=14)
plt.ylabel('y', fontsize=14)
plt.legend(fontsize=12)
plt.grid(True, linestyle=':', alpha=0.6)

# Set axis limits for better visualization (with some padding)
plt.xlim(min(x)-0.5, max(x)+0.5)
plt.ylim(min(y)-1, max(y)+1)

# Display
plt.tight_layout()
plt.show()

# Print results for verification
print(f"Fitting result: Intercept={intercept:.4f}, Slope={slope:.4f}")