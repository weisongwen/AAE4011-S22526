import numpy as np
import matplotlib.pyplot as plt

# 1. Data and initial settings
x_data = np.array([1.0, 2.0, 3.0])
y_data = np.array([1.6, 2.7, 4.5])
# Initial parameters
a_start = 1.0; b_start = 0.2; alpha = 0.04 
# Model definition: y = a * e^(bx)
def exponential_model(x, a, b):
    return a * np.exp(b * x)
# 2. Perform core calculations
# --- Step 1: Forward propagation (compute current predictions) ---
y_pred_initial = exponential_model(x_data, a_start, b_start)
# --- Step 2: Compute residuals (predictions - true values) ---
residuals = y_pred_initial - y_data
# --- Step 3: Compute gradients (core correction part) ---
# Gradient for a: Mean( (ae^bx - y) * e^bx )
grad_a_components = residuals * np.exp(b_start * x_data)
grad_a_mean = np.mean(grad_a_components) 
# Gradient for b: Mean( (ae^bx - y) * a * x * e^bx )
grad_b_components = residuals * a_start * x_data * np.exp(b_start * x_data)
grad_b_mean = np.mean(grad_b_components)
# --- Step 4: Parameter update (θ_new = θ_old - α * grad) ---
a_new = a_start - alpha * grad_a_mean
b_new = b_start - alpha * grad_b_mean

# ==============================================
# 3. Output calculation results
# ==============================================
print("="*50)
print("Single Iteration Calculation Results")
print("="*50)
print(f"Initial parameters: a = {a_start}, b = {b_start}")
print(f"Learning rate:     α = {alpha}")
print("-"*50)
print(f"Gradient calculations (mean):")
print(f"  ∇a (Mean) = {grad_a_mean:.4f}")
print(f"  ∇b (Mean) = {grad_b_mean:.4f}")
print("-"*50)
print(f"Parameter update process:")
print(f"  a_new = {a_start} - {alpha} × ({grad_a_mean:.4f}) = {a_new:.4f}")
print(f"  b_new = {b_start} - {alpha} × ({grad_b_mean:.4f}) = {b_new:.4f}")
print("="*50)
print(f"Final model: y = {a_new:.3f}e^({b_new:.3f}x)")

# ==============================================
# 4. Visualization plot
# ==============================================
plt.figure(figsize=(10, 6))

# 1. Plot data points
plt.scatter(x_data, y_data, color='black', s=120, label='Data Points', zorder=10)

# 2. Prepare x-coordinates for smooth curve
x_smooth = np.linspace(0.5, 3.5, 200)

# 3. Plot initial model (red dashed line)
y_initial_smooth = exponential_model(x_smooth, a_start, b_start)
plt.plot(x_smooth, y_initial_smooth, 'r--', linewidth=2, 
         label=f'Initial: $y={a_start}e^{{{b_start}x}}$')

# 4. Plot model after one iteration (green solid line)
y_new_smooth = exponential_model(x_smooth, a_new, b_new)
plt.plot(x_smooth, y_new_smooth, 'g-', linewidth=3, 
         label=f'After 1 Iteration: $y={a_new:.3f}e^{{{b_new:.3f}x}}$')

# 5. Plot predicted points movement
plt.scatter(x_data, y_pred_initial, color='red', alpha=0.5, marker='x', s=80, label='Initial Pred')
y_pred_new = exponential_model(x_data, a_new, b_new)
plt.scatter(x_data, y_pred_new, color='green', marker='o', s=80, label='New Pred')

# Draw arrows to show optimization direction
for i in range(len(x_data)):
    plt.arrow(x_data[i], y_pred_initial[i], 
              0, y_pred_new[i] - y_pred_initial[i], 
              head_width=0.05, head_length=0.1, fc='green', ec='green', alpha=0.6)

plt.title('Corrected Gradient Descent (Single Iteration)', fontsize=14, fontweight='bold')
plt.xlabel('x')
plt.ylabel('y')
plt.legend(loc='right', fontsize=10)
plt.grid(True, linestyle=':', alpha=0.6)
plt.xlim(0.5, 3.5)
plt.ylim(0, 5.5)

# Add parameter text box on the plot
text_str = '\n'.join((
    r'$\mathrm{Update\ Logic:}$',
    r'$a_{1} = 1.0 - 0.04 \times (-2.38) \approx 1.10$',
    r'$b_{1} = 0.2 - 0.04 \times (-6.24) \approx 0.45$'
))
props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
plt.text(0.05, 0.95, text_str, transform=plt.gca().transAxes, fontsize=11,
        verticalalignment='top', bbox=props)

plt.tight_layout()
plt.show()