import numpy as np
import matplotlib.pyplot as plt

# 3. Define the loss function
def loss_function(theta, x, y):
    """Compute the loss L(theta) = 1/2 * sum((theta * x_i - y_i)^2)"""
    return 0.5 * np.sum((theta * x - y) ** 2)

# 4. Define the gradient function
def gradient(theta, x, y):
    """Compute the gradient: ∇L(θ)^T = sum((θ * x_i - y_i) * x_i)"""
    return np.sum((theta * x - y) * x)

# 5. Define the Hessian function
def hessian(x):
    """Compute the Hessian: H = sum(x_i^2)"""
    return np.sum(x ** 2)

# 1. Define the dataset
x_data = np.array([1, 2, 3])
y_data = np.array([2, 4, 6])

# 2. Initialize parameters
theta_0 = 0.0  # Initial parameter value

# Calculate gradient at theta_0
grad = gradient(theta_0, x_data, y_data)

# Calculate Hessian
H = hessian(x_data)

# Perform Newton-Raphson update
theta_1 = theta_0 - grad / H

# 7. Calculate final loss
final_loss = loss_function(theta_1, x_data, y_data)

# 8. Generate predictions
y_pred = theta_1 * x_data

# 6. Perform the Newton-Raphson update step
print("=== Linear Data Fitting using Newton-Raphson Method ===\n")

print(f"Gradient at θ₀ = {theta_0}:")
print(f"∇L(θ)^T = ({theta_0}*1-2)*1 + ({theta_0}*2-4)*2 + ({theta_0}*3-6)*3")
print(f"∇L(θ)^T = {grad:.2f}")
print()

print(f"Hessian (sum of x_i^2):")
print(f"H = 1² + 2² + 3²")
print(f"H = {H:.2f}")
print()

print("Update using Newton-Raphson:")
print(f"θ₁ = θ₀ - H⁻¹ ∇L(θ)")
print(f"θ₁ = {theta_0} - ({grad}/{H})")
print(f"θ₁ = {theta_1:.2f}")
print()

print(f"Final model: y = {theta_1:.2f}x")
print(f"Final loss: L(θ₁) = {final_loss:.4f}")
print()


print("Predictions vs Actual values:")
for i in range(len(x_data)):
    print(f"x={x_data[i]}: y_pred={y_pred[i]:.2f}, y_actual={y_data[i]}")

# 9. Visualize the results
plt.figure(figsize=(10, 6))

# Plot the data points
plt.scatter(x_data, y_data, color='red', s=100, zorder=5, label='Data points')

# Plot the fitted line
x_range = np.linspace(0, 4, 100)
y_range = theta_1 * x_range
plt.plot(x_range, y_range, 'b-', linewidth=2, label=f'Fitted line: y = {theta_1:.2f}x')

# Add prediction points
plt.scatter(x_data, y_pred, color='green', s=80, zorder=5, label='Predictions')

# Customize the plot
plt.title('Linear Data Fitting: y = θx', fontsize=14, fontweight='bold')
plt.xlabel('x', fontsize=12)
plt.ylabel('y', fontsize=12)
plt.grid(True, alpha=0.3)
plt.legend(loc='right', fontsize=11)
plt.axhline(y=0, color='black', linewidth=0.5)
plt.axvline(x=0, color='black', linewidth=0.5)
plt.xlim(0, 3.5)
plt.ylim(0, 7)

# Add text box with results
text_str = f'Initial θ₀ = {theta_0}\nFinal θ₁ = {theta_1:.2f}\nLoss = {final_loss:.4f}'
plt.text(0.05, 0.95, text_str, transform=plt.gca().transAxes,
         fontsize=10, verticalalignment='top',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

plt.tight_layout()
plt.show()

# 10. Additional analysis
print("\n=== Additional Analysis ===")
print("Data points perfectly lie on the line y = 2x")
print("Therefore, the optimization found the exact solution in one iteration.")
print("This is expected because the model y = θx is linear in parameters")
print("and the loss function is quadratic, so Newton's method converges in one step.")