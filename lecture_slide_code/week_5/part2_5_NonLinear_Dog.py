import numpy as np
import matplotlib.pyplot as plt

# Model function
def model(x, theta):
    return np.exp(theta * x)

# 1. Setup Data and Model
x_data = np.array([1.0, 2.0, 3.0])
y_data = np.array([3.0, 8.0, 20.0])
theta_0 = 0.5
Delta = 0.5
# 2. Calculate Derivatives at theta_0
r = model(x_data, theta_0) - y_data 
# Jacobian: J = x * e^(theta*x)
J = x_data * model(x_data, theta_0)
# Gradient: g = J^T * r (Dot product)
grad = np.dot(J, r)  
# Hessian Approximation: H = J^T * J
H = np.dot(J, J)
# 3. Calculate Steps
p_gn = -grad / H
if np.ndim(grad) == 0: # If scalar
    numerator = grad**2
    denominator = np.sum((J * grad)**2)
else:
    numerator = np.dot(grad, grad)
    denominator = np.dot(np.dot(J, grad), np.dot(J, grad))

alpha = numerator / denominator
p_cp = -alpha * grad

norm_cp = np.linalg.norm(p_cp)
norm_gn = np.linalg.norm(p_gn)

step_len = 0.0
decision = ""

if norm_cp >= Delta:
    # Scenario 1: Cauchy point outside trust region
    step_len = Delta * (p_cp / norm_cp)
    decision = "Boundary (Cauchy Direction)"
elif norm_gn <= Delta:
    # Scenario 2: Gauss-Newton inside trust region
    step_len = p_gn
    decision = "Full Gauss-Newton"
else:
    # Scenario 3: Dogleg intersection
    step_len = Delta * np.sign(p_gn)
    decision = "Dogleg Intersection"

theta_new = theta_0 + step_len

# --- Output Results to Console ---
print(f"Gradient: {grad:.2f}")
print(f"Step GN:  {p_gn:.3f}")
print(f"Step CP:  {p_cp:.3f}")
print(f"Decision: {decision}")
print(f"Theta New: {theta_new:.4f}")

# --- Plotting ---
plt.figure(figsize=(10, 6))
x_plot = np.linspace(0.5, 3.5, 100)

plt.scatter(x_data, y_data, color='red', s=100, label='Data Points', zorder=5)
plt.plot(x_plot, model(x_plot, theta_0), 'b--', linewidth=2, label=f'Initial (theta={theta_0})')
plt.plot(x_plot, model(x_plot, theta_new), 'g-', linewidth=2, label=f'Updated (theta={theta_new:.2f})')

plt.title("Dogleg Optimization Step 1 (Fixed)", fontsize=14)
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid(True, linestyle=':', alpha=0.6)
plt.tight_layout()
plt.show()