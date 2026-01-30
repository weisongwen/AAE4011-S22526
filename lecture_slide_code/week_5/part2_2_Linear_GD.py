import numpy as np
import matplotlib.pyplot as plt

# Data points
x_data = np.array([1, 2, 3])
y_data = np.array([2, 4, 6])

# Initial parameters
a = 1.0
b = 0.0
theta = np.array([a, b])

# Learning rate
alpha = 0.18

# Calculate predictions with current parameters
y_pred = a * x_data + b

# Calculate gradients (correct formula)
# L = 1/(2*3) * Σ(ax_i + b - y_i)^2
# ∂L/∂a = 1/3 * Σ(ax_i + b - y_i) * x_i
# ∂L/∂b = 1/3 * Σ(ax_i + b - y_i)
grad_a = np.sum((y_pred - y_data) * x_data) / 3
grad_b = np.sum(y_pred - y_data) / 3

# Update parameters
a_new = a - alpha * grad_a
b_new = b - alpha * grad_b


print(f"Initial prediction: y = {a:.3f}x + {b:.3f}")
print(f"Gradient ∂L/∂a = {grad_a:.3f}")
print(f"Gradient ∂L/∂b = {grad_b:.3f}")

print(f"\nUpdated parameters:")
print(f"a_new = {a} - {alpha} × ({grad_a}) = {a_new:.3f}")
print(f"b_new = {b} - {alpha} × ({grad_b}) = {b_new:.3f}")
print(f"Updated prediction: y = {a_new:.3f}x + {b_new:.3f}")

# Visualization
plt.figure(figsize=(10, 6))

# Plot data points
plt.scatter(x_data, y_data, color='blue', s=100, label='Data points', zorder=5)

# Plot initial regression line
x_range = np.linspace(0, 4, 100)
y_initial = a * x_range + b
plt.plot(x_range, y_initial, 'r--', label=f'Initial: y = {a:.1f}x + {b:.1f}', alpha=0.7)

# Plot updated regression line
y_updated = a_new * x_range + b_new
plt.plot(x_range, y_updated, 'g-', label=f'Updated: y = {a_new:.3f}x + {b_new:.3f}', linewidth=2)

# Add prediction points for initial parameters
y_pred_initial = a * x_data + b
plt.scatter(x_data, y_pred_initial, color='red', s=80, marker='x', label='Initial predictions', zorder=5)

# Add prediction points for updated parameters
y_pred_updated = a_new * x_data + b_new
plt.scatter(x_data, y_pred_updated, color='green', s=80, marker='s', label='Updated predictions', zorder=5)

# Add error lines for initial predictions
for i in range(len(x_data)):
    plt.plot([x_data[i], x_data[i]], [y_data[i], y_pred_initial[i]], 'r:', alpha=0.5)

# Calculate and display MSE
mse_initial = np.mean((y_pred_initial - y_data) ** 2)
mse_updated = np.mean((y_pred_updated - y_data) ** 2)

plt.title('Linear Regression with Gradient Descent (One Step)', fontsize=14, fontweight='bold')
plt.xlabel('x', fontsize=12)
plt.ylabel('y', fontsize=12)
plt.legend(loc='right', fontsize=10)
plt.grid(True, alpha=0.3)
plt.axis([0, 4, 0, 7])

# Add text box with MSE values
textstr = f'MSE (Initial): {mse_initial:.4f}\nMSE (Updated): {mse_updated:.4f}'
props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
plt.text(0.05, 0.95, textstr, transform=plt.gca().transAxes, fontsize=10,
         verticalalignment='top', bbox=props)

# Add gradient information
plt.text(0.05, 0.05, f'Gradient: [∂L/∂a={grad_a:.3f}, ∂L/∂b={grad_b:.3f}]\nLearning rate: α={alpha}',
         transform=plt.gca().transAxes, fontsize=9, verticalalignment='bottom',
         bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

plt.tight_layout()
plt.show()

# Additional analysis
print(f"\nAdditional Analysis:")
print(f"Mean Squared Error (Initial): {mse_initial:.4f}")
print(f"Mean Squared Error (Updated): {mse_updated:.4f}")
print(f"Error reduction: {100*(mse_initial-mse_updated)/mse_initial:.2f}%")

























# import numpy as np
# import matplotlib.pyplot as plt

# # Data points
# x_data = np.array([1, 2, 3])
# y_data = np.array([2, 4, 6])

# # Initial parameters
# a = 1.0
# b = 0.0

# # Learning rate
# alpha = 0.01

# # Number of iterations
# iterations = 1000

# # Store history for visualization
# a_history = [a]
# b_history = [b]
# mse_history = []
# gradient_history = []

# print("Starting Gradient Descent Optimization...")
# print(f"Initial parameters: a = {a:.4f}, b = {b:.4f}")
# print(f"Learning rate: α = {alpha}")
# print(f"Number of iterations: {iterations}")
# print("-" * 50)

# # Perform multiple iterations
# for i in range(iterations):
#     # Calculate predictions with current parameters
#     y_pred = a * x_data + b
    
#     # Calculate gradients
#     grad_a = np.sum((y_pred - y_data) * x_data) / len(x_data)
#     grad_b = np.sum(y_pred - y_data) / len(x_data)
    
#     # Update parameters
#     a = a - alpha * grad_a
#     b = b - alpha * grad_b
    
#     # Calculate MSE
#     mse = np.mean((y_pred - y_data) ** 2)
    
#     # Store history
#     a_history.append(a)
#     b_history.append(b)
#     mse_history.append(mse)
#     gradient_history.append((grad_a, grad_b))
    
#     # Print progress every 100 iterations
#     if (i + 1) % 100 == 0:
#         print(f"Iteration {i+1:4d}: a = {a:.6f}, b = {b:.6f}, MSE = {mse:.6f}, "
#               f"Gradient = [{grad_a:.6f}, {grad_b:.6f}]")

# print("-" * 50)
# print("Optimization Complete!")
# print(f"Final parameters: a = {a:.6f}, b = {b:.6f}")
# print(f"Final MSE: {mse_history[-1]:.6f}")

# # Calculate analytical solution (closed-form)
# X = np.vstack([x_data, np.ones(len(x_data))]).T
# analytical_solution = np.linalg.inv(X.T @ X) @ X.T @ y_data
# a_analytic, b_analytic = analytical_solution
# print(f"Analytical solution (closed-form): a = {a_analytic:.6f}, b = {b_analytic:.6f}")

# # Create figure for visualization
# fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# # 1. Plot data and regression lines
# ax1 = axes[0, 0]
# ax1.scatter(x_data, y_data, color='blue', s=100, label='Data points', zorder=5)

# # Plot initial, final, and analytical regression lines
# x_range = np.linspace(0, 4, 100)
# ax1.plot(x_range, a_history[0] * x_range + b_history[0], 'r--', 
#          label=f'Initial: y = {a_history[0]:.2f}x + {b_history[0]:.2f}', alpha=0.7)
# ax1.plot(x_range, a_history[-1] * x_range + b_history[-1], 'g-', 
#          label=f'Final (GD): y = {a_history[-1]:.4f}x + {b_history[-1]:.4f}', linewidth=2)
# ax1.plot(x_range, a_analytic * x_range + b_analytic, 'b:', 
#          label=f'Analytical: y = {a_analytic:.4f}x + {b_analytic:.4f}', linewidth=2)

# ax1.set_xlabel('x', fontsize=12)
# ax1.set_ylabel('y', fontsize=12)
# ax1.set_title('Linear Regression: Data and Fitted Lines', fontsize=14, fontweight='bold')
# ax1.legend(fontsize=10)
# ax1.grid(True, alpha=0.3)
# ax1.axis([0, 4, 0, 7])

# # 2. Plot MSE over iterations
# ax2 = axes[0, 1]
# ax2.plot(range(1, len(mse_history) + 1), mse_history, 'b-', linewidth=2)
# ax2.set_xlabel('Iteration', fontsize=12)
# ax2.set_ylabel('Mean Squared Error (MSE)', fontsize=12)
# ax2.set_title('MSE vs. Iterations', fontsize=14, fontweight='bold')
# ax2.grid(True, alpha=0.3)
# ax2.set_xscale('log')  # Log scale for better visualization
# ax2.set_yscale('log')

# # Add convergence information
# final_mse = mse_history[-1]
# initial_mse = mse_history[0]
# convergence_ratio = final_mse / initial_mse
# ax2.text(0.05, 0.95, f'MSE reduction: {100*(1-convergence_ratio):.1f}%', 
#          transform=ax2.transAxes, fontsize=10, verticalalignment='top',
#          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# # 3. Plot parameter evolution
# ax3 = axes[1, 0]
# iterations_to_plot = min(100, len(a_history))
# iter_range = range(iterations_to_plot)

# ax3.plot(iter_range, a_history[:iterations_to_plot], 'r-', label='Parameter a', linewidth=2)
# ax3.plot(iter_range, b_history[:iterations_to_plot], 'b-', label='Parameter b', linewidth=2)

# # Add horizontal lines for analytical solution
# ax3.axhline(y=a_analytic, color='r', linestyle=':', alpha=0.7, label=f'Analytical a = {a_analytic:.4f}')
# ax3.axhline(y=b_analytic, color='b', linestyle=':', alpha=0.7, label=f'Analytical b = {b_analytic:.4f}')

# ax3.set_xlabel('Iteration', fontsize=12)
# ax3.set_ylabel('Parameter Value', fontsize=12)
# ax3.set_title('Parameter Evolution (First 100 Iterations)', fontsize=14, fontweight='bold')
# ax3.legend(fontsize=10)
# ax3.grid(True, alpha=0.3)

# # 4. Plot gradient magnitude over time
# ax4 = axes[1, 1]
# grad_magnitude = [np.sqrt(g[0]**2 + g[1]**2) for g in gradient_history]
# ax4.plot(range(len(grad_magnitude)), grad_magnitude, 'purple', linewidth=2)
# ax4.set_xlabel('Iteration', fontsize=12)
# ax4.set_ylabel('Gradient Magnitude', fontsize=12)
# ax4.set_title('Gradient Magnitude vs. Iterations', fontsize=14, fontweight='bold')
# ax4.grid(True, alpha=0.3)
# ax4.set_yscale('log')

# # Add text box with optimization summary
# summary_text = f"""
# Optimization Summary:
# - Initial: y = {a_history[0]:.2f}x + {b_history[0]:.2f}
# - Final:   y = {a_history[-1]:.4f}x + {b_history[-1]:.4f}
# - Analytical: y = {a_analytic:.4f}x + {b_analytic:.4f}
# - MSE reduction: {100*(1-final_mse/initial_mse):.1f}%
# - Final gradient: [{gradient_history[-1][0]:.6f}, {gradient_history[-1][1]:.6f}]
# """

# plt.figtext(0.5, 0.01, summary_text, ha='center', fontsize=10, 
#             bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))

# plt.tight_layout(rect=[0, 0.05, 1, 0.95])
# plt.show()

# # Additional analysis
# print("\n" + "="*60)
# print("ADDITIONAL ANALYSIS")
# print("="*60)

# # Convergence analysis
# print(f"\n1. Convergence Analysis:")
# print(f"   - Initial MSE: {initial_mse:.6f}")
# print(f"   - Final MSE: {final_mse:.6f}")
# print(f"   - Reduction: {100*(1-final_mse/initial_mse):.2f}%")

# # Parameter error
# print(f"\n2. Parameter Error (vs Analytical Solution):")
# print(f"   - a error: {abs(a_history[-1] - a_analytic):.6f} ({100*abs(a_history[-1] - a_analytic)/abs(a_analytic):.2f}%)")
# print(f"   - b error: {abs(b_history[-1] - b_analytic):.6f} ({100*abs(b_history[-1] - b_analytic)/abs(b_analytic):.2f}%)")

# # Learning rate analysis
# print(f"\n3. Learning Rate Analysis:")
# print(f"   - Current learning rate: {alpha}")
# print(f"   - Gradient magnitude at start: {grad_magnitude[0]:.6f}")
# print(f"   - Gradient magnitude at end: {grad_magnitude[-1]:.6f}")
# print(f"   - Gradient reduction: {100*(1-grad_magnitude[-1]/grad_magnitude[0]):.2f}%")

# # Final comparison
# print("\n" + "="*60)
# print("FINAL COMPARISON")
# print("="*60)

# # Predict with final parameters
# y_pred_final = a_history[-1] * x_data + b_history[-1]
# y_pred_analytic = a_analytic * x_data + b_analytic

# print(f"\nPredictions:")
# print(f"{'x':>5} {'y_actual':>10} {'y_GD':>10} {'y_Analytic':>12} {'Error_GD':>10} {'Error_Analytic':>14}")
# print("-" * 70)
# for i in range(len(x_data)):
#     error_gd = abs(y_pred_final[i] - y_data[i])
#     error_analytic = abs(y_pred_analytic[i] - y_data[i])
#     print(f"{x_data[i]:5.1f} {y_data[i]:10.1f} {y_pred_final[i]:10.4f} {y_pred_analytic[i]:12.4f} "
#           f"{error_gd:10.4f} {error_analytic:14.4f}")

# print("\n" + "="*60)
# print("SUMMARY")
# print("="*60)
# print(f"• Gradient descent successfully converged to near-optimal solution")
# print(f"• After {iterations} iterations, parameters are very close to analytical solution")
# print(f"• MSE decreased from {initial_mse:.4f} to {final_mse:.4f}")
# print(f"• For this simple problem, gradient descent works well with learning rate {alpha}")
# print(f"• To improve: adaptive learning rates, early stopping, or momentum could be added")