import numpy as np
import matplotlib.pyplot as plt

# Data points
X = np.array([1, 2, 3])
Y = np.array([3, 5, 7])

# Initial parameters
theta = np.array([0.0, 0.0])  # [a, b]
theta_initial = theta.copy()

# Gauss-Newton method
def gauss_newton(X, Y, theta):
    # Construct Jacobian matrix
    J = np.column_stack([-X, -np.ones_like(X)])
    # Compute residuals
    residuals = Y - (theta[0] * X + theta[1])
    # Compute gradient J^T * r
    gradient = J.T @ residuals
    # Compute Hessian approximation J^T * J
    hessian_approx = J.T @ J
    # Parameter update
    delta_theta = -np.linalg.inv(hessian_approx) @ gradient
    return theta + delta_theta

print("Initial parameters: a =", theta[0], ", b =", theta[1])

# One-step Gauss-Newton update
theta_new = gauss_newton(X, Y, theta)
print("\nUpdated parameters: a =", theta_new[0], ", b =", theta_new[1])

# Plotting
plt.figure(figsize=(10, 6))
x_plot = np.linspace(0.5, 3.5, 100)

# Initial fit line
y_initial = theta_initial[0] * x_plot + theta_initial[1]

# Final fit line
y_final = theta_new[0] * x_plot + theta_new[1]

# Plot data points
plt.scatter(X, Y, color='red', s=100, zorder=5, label='Data points')

# Plot initial fit line
plt.plot(x_plot, y_initial, 'b--', linewidth=2, alpha=0.7, 
         label=f'Initial fit: y = {theta_initial[0]:.1f}x + {theta_initial[1]:.1f}')

# Plot final fit line
plt.plot(x_plot, y_final, 'g-', linewidth=3, 
         label=f'Final fit: y = {theta_new[0]:.1f}x + {theta_new[1]:.1f}')

# Label data points
plt.text(1.05, 3.1, f'(1, 3)', fontsize=10, ha='left')
plt.text(2.05, 4.9, f'(2, 5)', fontsize=10, ha='left') 
plt.text(3.05, 6.9, f'(3, 7)', fontsize=10, ha='left')

plt.xlabel('x', fontsize=12)
plt.ylabel('y', fontsize=12)
plt.title('Linear Data Fitting using Gauss-Newton Method', fontsize=14, pad=15)

# Adjust legend position
plt.legend(loc='right', fontsize=11, framealpha=0.9)

plt.grid(True, alpha=0.3)
plt.axis([0.5, 3.5, 2, 8])
plt.tight_layout()

# Add text info box
textstr = '\n'.join((
    f'Initial guess: a={theta_initial[0]:.1f}, b={theta_initial[1]:.1f}',
    f'After 1 iteration:',
    f'  a = {theta_new[0]:.2f}',
    f'  b = {theta_new[1]:.2f}',
    f'Perfect fit achieved!'
))

plt.text(0.55, 7.5, textstr, fontsize=10, 
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
         verticalalignment='top')

plt.show()

# Verify residuals
print("\nVerify final residuals:")
y_pred = theta_new[0] * X + theta_new[1]
residuals_final = Y - y_pred
print("Predictions:", y_pred)
print("Residuals:", residuals_final)
print("Sum of squared residuals:", np.sum(residuals_final**2))