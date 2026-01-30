import numpy as np
import matplotlib.pyplot as plt

# Data points (x, y)
x_data = np.array([1.0, 2.0, 3.0])
y_data = np.array([1.0, 4.0, 9.0]); N = len(x_data)
# Hyperparameters
alpha = 0.2; beta1 = 0.9
beta2 = 0.999; epsilon = 1e-8; epochs = 1
# Initialization
w = 0.5; m = 0.0; v = 0.0; t = 0
# Storage for plotting
w_history = [w]; loss_history = []
# ADAM optimization
for epoch in range(epochs):
    for i in range(N):
        t += 1
        # Compute prediction and gradient
        pred = w * (x_data[i] ** 2)
        error = pred - y_data[i]
        g = (x_data[i] ** 2) * error
        # Update moments
        m = beta1 * m + (1 - beta1) * g
        v = beta2 * v + (1 - beta2) * (g ** 2)
        # Bias correction
        m_hat = m / (1 - beta1 ** t)
        v_hat = v / (1 - beta2 ** t)
        # Update parameter
        w = w - alpha * m_hat / (np.sqrt(v_hat) + epsilon)
    
    # Store for plotting
    w_history.append(w)
    loss = 0.5 * np.mean((w * (x_data ** 2) - y_data) ** 2)
    loss_history.append(loss)

print(f"Final w after {epochs} epochs: {w:.4f}")

# Plotting
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

# Plot 1: Data points and fitted curve
x_plot = np.linspace(0.5, 3.5, 100)
y_plot = w * (x_plot ** 2)
ax1.scatter(x_data, y_data, color='red', s=100, zorder=5, label='Data points')
ax1.plot(x_plot, y_plot, 'b-', linewidth=2, label=f'Fitted curve: y = {w:.4f}·x²')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_title('Non-Linear Regression with ADAM: y = w·x²')
ax1.grid(True, alpha=0.3)
ax1.legend()
ax1.set_xlim(0.5, 3.5)
ax1.set_ylim(0, 10)

# Plot 2: Loss function over epochs
ax2.plot(range(len(loss_history)), loss_history, marker='o', color='orange')
ax2.set_xlabel('Epoch')
ax2.set_ylabel('Loss')
ax2.set_title('Loss Function Decrease')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()