import numpy as np
import matplotlib.pyplot as plt
# Data
x = np.array([1, 2, 3], dtype=float)
y = np.array([2, 4, 6], dtype=float); N = len(x)
# Hyperparameters
alpha = 0.5; beta1 = 0.9; beta2 = 0.999
eps = 1e-8; epochs = 1
# Initialization
w = 0.5; m = 0.0; v = 0.0; t = 0
# Storage for plotting
w_history = [w]; loss_history = []
# ADAM optimization
for epoch in range(epochs):
    for i in range(N):
        t += 1
        # Gradient for sample (x[i], y[i])
        pred = w * x[i]
        g = x[i] * (pred - y[i])
        
        # Update moments
        m = beta1 * m + (1 - beta1) * g
        v = beta2 * v + (1 - beta2) * (g**2)
        
        # Bias correction
        m_hat = m / (1 - beta1**t)
        v_hat = v / (1 - beta2**t)
        
        # Update w
        w = w - alpha * m_hat / (np.sqrt(v_hat) + eps)
        
    # Store for plotting
    w_history.append(w)
    loss = 0.5 * np.mean((w * x - y)**2)
    loss_history.append(loss)

print(f"Final w after {epochs} epochs: {w:.4f}")

# Plotting
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))

# Plot 1: Data points and fitted line
x_line = np.linspace(0, 4, 100)
y_line = w * x_line
ax1.scatter(x, y, color='red', s=100, zorder=5, label='Data points')
ax1.plot(x_line, y_line, 'b-', linewidth=2, label=f'Fitted line: y = {w:.4f}x')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_title('Linear Regression with ADAM')
ax1.grid(True, alpha=0.3)
ax1.legend()
ax1.set_xlim(0, 4)
ax1.set_ylim(0, 7)

# Plot 2: Loss function over epochs
ax2.plot(range(len(loss_history)), loss_history, marker='o', color='orange')
ax2.set_xlabel('Epoch')
ax2.set_ylabel('Loss')
ax2.set_title('Loss Function Decrease')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()