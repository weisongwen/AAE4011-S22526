import numpy as np
import matplotlib.pyplot as plt

# Model function: y = e^(theta * x)
def model(x, theta):
    return np.exp(theta * x)

# 2. Define Derivatives
def compute_gradient(x, y, theta):
    # Gradient = Sum( (e^tx - y) * (x * e^tx) )
    predictions = model(x, theta)
    residuals = predictions - y
    gradients = residuals * (x * predictions)
    return np.sum(gradients)

def compute_hessian(x, y, theta):
    # Hessian = Sum( (x*e^tx)^2 + (e^tx - y)*(x^2*e^tx) )
    predictions = model(x, theta)
    residuals = predictions - y
    
    # Term 1: Gauss-Newton approximation term (Always positive)
    term1 = (x * predictions) ** 2
    # Term 2: Residual term (Can be negative if residuals are large negative)
    term2 = residuals * (x**2 * predictions)
    
    return np.sum(term1 + term2)

# 1. Define NEW Data and Model
x_data = np.array([0.5, 1.0, 1.5])
y_data = np.array([1.35, 1.82, 2.46]) # NEW DATA POINTS
theta_0 = 0.5

# 3. Numerical Calculation
grad_val = compute_gradient(x_data, y_data, theta_0)
hess_val = compute_hessian(x_data, y_data, theta_0)

# Calculate the update theta_1
theta_1 = theta_0 - (grad_val / hess_val)

# 4. Plotting
x_plot = np.linspace(0, 2.0, 100)
y_initial = model(x_plot, theta_0)
y_updated = model(x_plot, theta_1)

# Print results to console
print(f"--- Calculation Results with NEW DATA ---")
print(f"Initial Guess theta_0: {theta_0}")
print(f"Gradient: {grad_val:.4f} (Negative, meaning we need to increase theta)")
print(f"Hessian:  {hess_val:.4f}  (Positive and large, implying stability)")
print(f"Update Step (-G/H): {-grad_val/hess_val:.4f}")
print(f"New Theta_1: {theta_1:.4f}")

plt.figure(figsize=(10, 6))

# Plot Data Points
plt.scatter(x_data, y_data, color='red', s=100, label='Data Points', zorder=5)

# Plot Initial Model
plt.plot(x_plot, y_initial, color='blue', linestyle='--', linewidth=2, 
         label=f'Initial Guess (theta={theta_0})')

# Plot Updated Model
plt.plot(x_plot, y_updated, color='green', linewidth=2, 
         label=f'Updated Model (theta={theta_1:.3f})')

# Formatting
plt.title(f'Newton\'s Method Success: Hessian is Positive ({hess_val:.2f})', fontsize=14)
plt.xlabel('x', fontsize=12)
plt.ylabel('y', fontsize=12)
plt.legend(loc='right', fontsize=12)
plt.grid(True, alpha=0.3)

# Add text annotation
plt.text(0.05, 3.0, 
         f"Update Step:\n"
         f"Theta_0 = {theta_0}\n"
         f"Theta_1 = {theta_0} - ({grad_val:.2f} / {hess_val:.2f})\n"
         f"        = {theta_1:.3f}\n"
         f"Result: The green curve moves closer to the red dots.", 
         bbox=dict(facecolor='lightgreen', alpha=0.2), fontsize=11)

plt.show()