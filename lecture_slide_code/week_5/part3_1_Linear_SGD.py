import matplotlib.pyplot as plt
import numpy as np
# from tabulate import tabulate

# Data
x_data = [1, 2, 3]
y_data = [2, 4, 6]

# Initial parameters
w = 0.5
b = 0
alpha = 0.1  # learning rate

# Store results for table
results = []

# Iterate through samples
for i in range(3):
    x = x_data[i]
    y = y_data[i]
    
    # Forward pass
    y_hat = w * x + b
    error = y - y_hat
    
    # Compute gradients
    dw = -2 * x * error
    db = -2 * error
    
    # Store results before update
    results.append([
        f"({x}, {y})",
        f"y = {w:.3f}x + {b:.3f}",
        f"{y_hat:.3f}",
        f"{error:.3f}",
        f"dw = {dw:.3f}",
        f"db = {db:.3f}",
        f"w={w:.3f}, b={b:.3f}"
    ])
    
    # Update parameters
    w = w - alpha * dw
    b = b - alpha * db

# Print table
headers = ["Sample (x,y)", "Current Model", "Pred ŷ", "Error (y-ŷ)", "Gradient dw", "Gradient db", "Updated Params"]
print("SGD Iteration Process")
print("="*100)
# print(tabulate(results, headers=headers, tablefmt="grid"))

# Plotting
plt.figure(figsize=(12, 5))

# Plot 1: Data points and regression lines
plt.subplot(1, 2, 1)
x_range = np.linspace(0, 4, 100)

# Reset parameters for plotting progression
w_vals = [0.5, 0.8, 1.64, 1.856]
b_vals = [0, 0.3, 0.72, 0.792]
labels = ["Initial: w=0.5, b=0", 
          "After sample 1: w=0.8, b=0.3", 
          "After sample 2: w=1.64, b=0.72", 
          "After sample 3: w=1.856, b=0.792"]

colors = ['gray', 'orange', 'green', 'blue']
for i, (w_plot, b_plot) in enumerate(zip(w_vals, b_vals)):
    y_range = w_plot * x_range + b_plot
    plt.plot(x_range, y_range, label=labels[i], color=colors[i], alpha=0.7)

# Plot data points
plt.scatter(x_data, y_data, color='red', s=100, zorder=5, label='Data points')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Linear Regression with SGD Updates')
plt.legend()
plt.grid(True, alpha=0.3)
plt.xlim(0, 4)
plt.ylim(0, 7)

# Plot 2: Loss function
plt.subplot(1, 2, 2)
# Calculate MSE for each parameter set
w_grid = np.linspace(0, 3, 50)
b_grid = np.linspace(-1, 2, 50)
W, B = np.meshgrid(w_grid, b_grid)

# Mean Squared Error
MSE = np.zeros_like(W)
for x, y in zip(x_data, y_data):
    MSE += (y - (W * x + B)) ** 2
MSE /= len(x_data)

# Contour plot
contour = plt.contour(W, B, MSE, levels=20, alpha=0.6)
plt.clabel(contour, inline=True, fontsize=8)

# Plot parameter updates
plt.plot(w_vals, b_vals, 'ro-', markersize=8, label='SGD updates')
for i, (w_val, b_val) in enumerate(zip(w_vals, b_vals)):
    plt.annotate(f'Step {i}', (w_val, b_val), xytext=(5, 5), 
                 textcoords='offset points', fontsize=9)

plt.xlabel('Weight (w)')
plt.ylabel('Bias (b)')
plt.title('Loss Function Contour & Optimization Path')
plt.legend()
plt.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('sgd_optimization.png', dpi=300, bbox_inches='tight')
plt.show()

# Print mathematical formulas
print("\n" + "="*100)
print("MATHEMATICAL FORMULAS")
print("="*100)
print("Loss Function: L = Σ(y - (wx + b))²")
print("Gradient w.r.t w: ∂L/∂w = -2 * Σ[x * (y - (wx + b))]")
print("Gradient w.r.t b: ∂L/∂b = -2 * Σ[y - (wx + b)]")
print("Update Rule: w_new = w_old - α * ∂L/∂w")
print("              b_new = b_old - α * ∂L/∂b")
print("\nWhere:")
print("  α = learning rate = 0.1")
print("  Initial values: w₀ = 0.5, b₀ = 0")
print("  Ground truth: w* = 2.0, b* = 0.0")