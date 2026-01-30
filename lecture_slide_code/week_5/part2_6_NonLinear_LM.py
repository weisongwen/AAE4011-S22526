import numpy as np
import matplotlib.pyplot as plt

# 1. Data & Parameters
x = np.array([1.0, 2.0, 3.0])
y = np.array([3.0, 8.0, 20.0])
theta = np.array([0.5, 0.5]) # Initial guess [a, b]
lam = 180                   # Damping factor lambda
# 2. Calculate Residuals & Jacobian
exp_bx = np.exp(theta[1] * x)
r = y - theta[0] * exp_bx
J = np.column_stack((-exp_bx, -theta[0] * x * exp_bx))
# 3. LM Update Step: (J.T*J + lambda*I) * delta = -J.T*r
A = J.T @ J + lam * np.eye(2)
g = -J.T @ r
delta_theta = np.linalg.solve(A, g)
theta_new = theta + delta_theta

# Print results to verify against image
print(f"Residuals r: {r}")
print(f"Delta theta: {delta_theta}")
print(f"New theta:   {theta_new}")

# 4. Plotting
x_line = np.linspace(0.5, 3.5, 100)
y_init = theta[0] * np.exp(theta[1] * x_line)
y_new  = theta_new[0] * np.exp(theta_new[1] * x_line)

plt.figure(figsize=(8, 5))
plt.scatter(x, y, color='red', label='Data Points', zorder=5)
plt.plot(x_line, y_init, 'g--', label=f'Initial: $y={theta[0]}e^{{{theta[1]}x}}$')
plt.plot(x_line, y_new, 'b-', label=f'LM Step 1: $y={theta_new[0]:.2f}e^{{{theta_new[1]:.2f}x}}$')
plt.title('LM Algorithm - First Iteration')
plt.xlabel('x'); plt.ylabel('y')
plt.legend(); plt.grid(True)
plt.show()