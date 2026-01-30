import numpy as np
import matplotlib.pyplot as plt
# Data
x = np.array([1, 2, 3])
y = np.array([1.8, 4.1, 5.9])
# Initial parameters
theta0 = 1.0
delta = 2.0  # Trust region radius
# Step 1: Calculate residuals, Jacobian, gradient and Hessian at theta0
r = theta0 * x - y
J = x
gradient = J.T @ r
Hessian = J.T @ J
# Step 2: Calculate Gauss-Newton step
p_GN = -gradient / Hessian
# Step 3: Check if step is within trust region
if np.abs(p_GN) <= delta:
    theta1 = theta0 + p_GN
else:
    # In this case, we would use Cauchy point or dogleg method
    # For simplicity, we take scaled step
    theta1 = theta0 + delta * p_GN / np.abs(p_GN)

print(f"Updated parameter: theta = {theta1:.3f}")






# Step 4: Plot results
fig, ax = plt.subplots(figsize=(10, 6))

# Original data points
ax.scatter(x, y, color='red', s=100, label='Data points', zorder=5)

# Initial model (theta0)
x_line = np.linspace(0.5, 3.5, 100)
y_init = theta0 * x_line
y_final = theta1 * x_line

ax.plot(x_line, y_init, 'b--', linewidth=2, label=f'Initial model: y = {theta0:.1f}x')
ax.plot(x_line, y_final, 'g-', linewidth=2, label=f'Fitted model: y = {theta1:.3f}x')

# Highlight the Gauss-Newton step
ax.axvline(x=0, color='gray', linestyle=':', alpha=0.5)
ax.axhline(y=0, color='gray', linestyle=':', alpha=0.5)

# Trust region visualization
if np.abs(p_GN) <= delta:
    ax.fill_between([theta0 - delta, theta0 + delta], -2, 10, alpha=0.1, color='green',
                   label=f'Trust region (Δ={delta})')
else:
    ax.fill_between([theta0 - delta, theta0 + delta], -2, 10, alpha=0.1, color='red',
                   label=f'Trust region (Δ={delta})')

# Formatting
ax.set_xlabel('x', fontsize=12)
ax.set_ylabel('y', fontsize=12)
ax.set_title('Linear Fitting with Gauss-Newton Method', fontsize=14)
ax.grid(True, alpha=0.3)
ax.legend(loc='right')
ax.set_xlim([0.5, 3.5])
ax.set_ylim([0, 8])

# Add text box with algorithm details
textstr = '\n'.join((
    f'Initial θ₀: {theta0:.1f}',
    f'Gradient: {gradient:.3f}',
    f'Hessian: {Hessian:.3f}',
    f'Gauss-Newton step: {p_GN:.3f}',
    f'Trust region radius Δ: {delta}',
    f'Final θ₁: {theta1:.3f}'))
props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=10,
        verticalalignment='top', bbox=props)

plt.tight_layout()
plt.show()

# Calculate RMSE for both models
rmse_init = np.sqrt(np.mean((theta0 * x - y)**2))
rmse_final = np.sqrt(np.mean((theta1 * x - y)**2))
print(f"\nRMSE (initial model): {rmse_init:.3f}")
print(f"RMSE (fitted model): {rmse_final:.3f}")