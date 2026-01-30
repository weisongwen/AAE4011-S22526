import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# 1. Prepare data
x_data = np.array([0, 1, 2]); y_data = np.array([2.0, 3.3, 5.4])
# 2. Initial guess
theta = np.array([1.0, 0.0]) # [a, b]
def gauss_newton_solve(x, y, theta_init, tol=1e-6, max_iter=10):
    theta = theta_init.copy(); history = []
    # Record initial state
    rss = np.sum((y - theta[0]*np.exp(theta[1]*x))**2)
    history.append({'iter': 0, 'a': theta[0], 'b': theta[1], 'RSS': rss})
    print(f"{'Iter':<5} | {'a':<10} | {'b':<10} | {'RSS':<15}")
    print(f"{0:<5} | {theta[0]:<10.6f} | {theta[1]:<10.6f} | {rss:<15.6f}")
    for k in range(max_iter):
        a, b = theta
        # Step 1: Compute predictions and residuals
        y_pred = a * np.exp(b * x); r = y - y_pred 
        # Step 2: Compute Jacobian matrix J
        J = np.zeros((len(x), 2))
        J[:, 0] = -np.exp(b * x); J[:, 1] = -x * a * np.exp(b * x)
        # Step 3: Matrix operations
        JTJ = J.T @ J; JTr = J.T @ r
        # Step 4: Parameter update
        try:
            update_term = np.linalg.solve(JTJ, JTr)
        except np.linalg.LinAlgError:
            break
        # Perform subtraction
        theta_new = theta - update_term
        # Compute new error
        new_rss = np.sum((y - theta_new[0]*np.exp(theta_new[1]*x))**2)
        # Record and print
        theta = theta_new
        history.append({'iter': k+1, 'a': theta[0], 'b': theta[1], 'RSS': new_rss})
        print(f"{k+1:<5} | {theta[0]:<10.6f} | {theta[1]:<10.6f} | {new_rss:<15.6f}")

        # Check convergence
        if np.linalg.norm(update_term) < tol:
            print("Converged!")
            break
            
    return theta, pd.DataFrame(history)

# Run solver
theta_final, df = gauss_newton_solve(x_data, y_data, theta)

# --- Plotting ---
x_plot = np.linspace(-0.5, 2.5, 100)
y_final = theta_final[0] * np.exp(theta_final[1] * x_plot)

plt.figure(figsize=(8, 5))
plt.scatter(x_data, y_data, color='red', s=80, label='Data Points', zorder=5)
plt.plot(x_plot, y_final, 'b-', linewidth=2, label=f'Final Fit: {theta_final[0]:.2f}e^({theta_final[1]:.2f}x)')

# Plot initial guess
y_init = 1.0 * np.exp(0.0 * x_plot)
plt.plot(x_plot, y_init, 'k--', label='Initial (Iter 0)', alpha=0.5)

plt.title("Corrected Gauss-Newton Fitting")
plt.legend()
plt.grid(True)
plt.show()