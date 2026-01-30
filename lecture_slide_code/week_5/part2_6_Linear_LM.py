import numpy as np
import matplotlib.pyplot as plt

# Data points derived from J^T J and J^T r
x = np.array([1, 2, 3])
y = np.array([-5, 0, 8])
# Initial parameters
theta0 = np.array([1.0, 1.0])
# Compute Jacobian and residuals at theta0
# Model: y = theta[0] * x + theta[1]
pred = theta0[0] * x + theta0[1]
r = y - pred  # residuals
# Jacobian J:
# For each data point: [-x_i, -1]
J = np.column_stack([-x, -np.ones_like(x)])
# Compute J^T J and J^T r
J_T_J = J.T @ J
J_T_r = J.T @ r
# Levenberg-Marquardt update with lambda = 0.01
lambda_lm = 0.01
A = J_T_J + lambda_lm * np.eye(2)
b = -J_T_r
delta_theta = np.linalg.solve(A, b)
theta1 = theta0 + delta_theta

print("Delta theta:", delta_theta)
print("Updated theta:", theta1)

# Plot data points and fit lines
plt.scatter(x, y, color='red', label='Data points')
x_line = np.linspace(0, 4, 100)
y_init = theta0[0] * x_line + theta0[1]
y_updated = theta1[0] * x_line + theta1[1]
plt.plot(x_line, y_init, 'b--', label='Initial fit: y = {:.2f}x + {:.2f}'.format(theta0[0], theta0[1]))
plt.plot(x_line, y_updated, 'g-', label='LM updated: y = {:.2f}x + {:.2f}'.format(theta1[0], theta1[1]))
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('Linear Data Fitting with Levenberg-Marquardt')
plt.grid(True)
plt.show()