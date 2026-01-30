import numpy as np
import matplotlib.pyplot as plt

# 1. Data and Hyperparameters
x_data = np.array([1.0, 2.0, 3.0])
y_data = np.array([1.0, 4.0, 9.0])
w = 0.1          # Initial weight (w0)
alpha = 0.01     # Learning rate
w_history = [w]  # To store weights for visualization

print(f"{'Step':<5} | {'Sample':<8} | {'Pred':<8} | {'Error':<8} | {'Grad (dw)':<10} | {'New w':<8}")
print("-" * 65)

# 2. Iteration (SGD: Update w sample by sample)
for i, (x, y) in enumerate(zip(x_data, y_data)):
    # Forward pass: prediction
    y_pred = w * (x ** 2)
    
    # Calculate error
    error = y - y_pred
    
    # Backward pass: Gradient for single sample
    # Formula: dL/dw = -2 * x^2 * (y - wx^2)
    dw = -2 * (x ** 2) * error
    
    # Update weight
    w = w - alpha * dw
    w_history.append(w)
    
    print(f"{i+1:<5} | ({int(x)}, {int(y)})    | {y_pred:<8.3f} | {error:<8.3f} | {dw:<10.3f} | {w:<8.3f}")

# 3. Visualization
plt.figure(figsize=(10, 5))

# Plot 1: Data fitting
plt.subplot(1, 2, 1)
x_range = np.linspace(0, 3.5, 100)
plt.scatter(x_data, y_data, color='red', label='True Data Points', zorder=5)
# Initial Model
plt.plot(x_range, w_history[0] * x_range**2, 'g--', label=f'Initial Model (w={w_history[0]:.2f})')
# Final Model
plt.plot(x_range, w_history[-1] * x_range**2, 'b-', label=f'Final Model (w={w_history[-1]:.3f})')
plt.title("Non-Linear Data Fitting")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid(True)

# Plot 2: Weight update trajectory
plt.subplot(1, 2, 2)
plt.plot(range(len(w_history)), w_history, 'o-', color='purple')
plt.title("Weight Update Trajectory")
plt.xlabel("Step")
plt.ylabel("Weight (w)")
plt.xticks(range(len(w_history)))
plt.grid(True)

plt.tight_layout()
plt.show()