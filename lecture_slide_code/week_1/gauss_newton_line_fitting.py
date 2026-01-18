"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026
Week 1: Introduction of AI for Unmanned Autonomous Systems

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk
Address: R820, PolyU

This code is provided for educational purposes as part of the AAE4011 course.
All rights reserved.

Gauss-Newton Method for Line Fitting
This example demonstrates the Gauss-Newton optimization algorithm for fitting a line to noisy data.
"""

import numpy as np
import matplotlib.pyplot as plt

def generate_noisy_line_data(n_points=50, slope=2.0, intercept=1.0, noise_level=0.5):
    """
    Generate noisy data points along a line.
    
    Parameters:
    -----------
    n_points : int
        Number of data points to generate
    slope : float
        True slope of the line
    intercept : float
        True y-intercept of the line
    noise_level : float
        Standard deviation of Gaussian noise
    
    Returns:
    --------
    x : numpy array
        x-coordinates of data points
    y : numpy array
        y-coordinates of data points (with noise)
    """
    x = np.linspace(0, 10, n_points)
    y_true = slope * x + intercept
    noise = np.random.normal(0, noise_level, n_points)
    y = y_true + noise
    return x, y, y_true


def residual_function(params, x, y):
    """
    Compute residuals for line fitting.
    Residual = y - (a*x + b), where params = [a, b]
    
    Parameters:
    -----------
    params : numpy array
        Parameters [slope, intercept]
    x : numpy array
        x-coordinates
    y : numpy array
        y-coordinates (observed values)
    
    Returns:
    --------
    residuals : numpy array
        Residuals for each data point
    """
    slope, intercept = params
    y_predicted = slope * x + intercept
    residuals = y - y_predicted
    return residuals


def jacobian(params, x, y):
    """
    Compute Jacobian matrix for Gauss-Newton method.
    J[i, 0] = -x[i]  (derivative w.r.t. slope)
    J[i, 1] = -1     (derivative w.r.t. intercept)
    
    Parameters:
    -----------
    params : numpy array
        Parameters [slope, intercept]
    x : numpy array
        x-coordinates
    y : numpy array
        y-coordinates (not used, but kept for consistency)
    
    Returns:
    --------
    J : numpy array
        Jacobian matrix (n_points x 2)
    """
    n_points = len(x)
    J = np.zeros((n_points, 2))
    J[:, 0] = -x  # derivative w.r.t. slope
    J[:, 1] = -1  # derivative w.r.t. intercept
    return J


def gauss_newton_line_fitting(x, y, initial_params=None, max_iterations=100, tolerance=1e-6):
    """
    Fit a line to data using Gauss-Newton method.
    
    Parameters:
    -----------
    x : numpy array
        x-coordinates of data points
    y : numpy array
        y-coordinates of data points
    initial_params : numpy array, optional
        Initial guess [slope, intercept]. If None, uses simple linear regression estimate.
    max_iterations : int
        Maximum number of iterations
    tolerance : float
        Convergence tolerance
    
    Returns:
    --------
    params : numpy array
        Fitted parameters [slope, intercept]
    history : list
        History of parameter values and costs during optimization
    """
    if initial_params is None:
        # Simple initial guess using linear regression
        A = np.vstack([x, np.ones(len(x))]).T
        initial_params = np.linalg.lstsq(A, y, rcond=None)[0]
    
    params = initial_params.copy()
    history = []
    
    for iteration in range(max_iterations):
        # Compute residuals
        residuals = residual_function(params, x, y)
        cost = 0.5 * np.sum(residuals**2)
        
        # Store history
        history.append({
            'iteration': iteration,
            'params': params.copy(),
            'cost': cost
        })
        
        # Compute Jacobian
        J = jacobian(params, x, y)
        
        # Gauss-Newton update: (J^T * J) * delta = -J^T * r
        JTJ = J.T @ J
        JTr = J.T @ residuals
        
        # Solve for parameter update
        try:
            delta = np.linalg.solve(JTJ, -JTr)
        except np.linalg.LinAlgError:
            print(f"Warning: Singular matrix at iteration {iteration}")
            break
        
        # Update parameters
        params_new = params + delta
        
        # Check convergence
        if np.linalg.norm(delta) < tolerance:
            print(f"Converged after {iteration + 1} iterations")
            break
        
        params = params_new
    
    return params, history


def plot_results(x, y, y_true, params, history):
    """
    Plot the fitting results and optimization history.
    
    Parameters:
    -----------
    x : numpy array
        x-coordinates
    y : numpy array
        Observed y-coordinates (with noise)
    y_true : numpy array
        True y-coordinates (without noise)
    params : numpy array
        Fitted parameters [slope, intercept]
    history : list
        Optimization history
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Plot 1: Data and fitted line
    ax1 = axes[0]
    ax1.scatter(x, y, alpha=0.6, label='Noisy Data', color='blue', s=30)
    ax1.plot(x, y_true, 'g--', label='True Line', linewidth=2)
    
    # Fitted line
    slope, intercept = params
    y_fitted = slope * x + intercept
    ax1.plot(x, y_fitted, 'r-', label=f'Fitted Line (y={slope:.3f}x+{intercept:.3f})', linewidth=2)
    
    ax1.set_xlabel('x', fontsize=12)
    ax1.set_ylabel('y', fontsize=12)
    ax1.set_title('Gauss-Newton Line Fitting', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Cost function convergence
    ax2 = axes[1]
    iterations = [h['iteration'] for h in history]
    costs = [h['cost'] for h in history]
    ax2.plot(iterations, costs, 'b-o', markersize=4, linewidth=2)
    ax2.set_xlabel('Iteration', fontsize=12)
    ax2.set_ylabel('Cost (0.5 * sum of squared residuals)', fontsize=12)
    ax2.set_title('Convergence History', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_yscale('log')
    
    plt.tight_layout()
    plt.show()
    
    # Print results
    print("\n" + "="*50)
    print("Fitting Results:")
    print("="*50)
    print(f"True slope:      {2.0:.6f}")
    print(f"Fitted slope:    {slope:.6f}")
    print(f"Error:           {abs(slope - 2.0):.6f}")
    print(f"\nTrue intercept:  {1.0:.6f}")
    print(f"Fitted intercept: {intercept:.6f}")
    print(f"Error:           {abs(intercept - 1.0):.6f}")
    print(f"\nFinal cost:      {costs[-1]:.6f}")
    print(f"Number of iterations: {len(history)}")
    print("="*50)


def main():
    """
    Main function to demonstrate Gauss-Newton line fitting.
    """
    print("Gauss-Newton Method for Line Fitting")
    print("="*50)
    
    # Generate noisy line data
    print("\nGenerating noisy line data...")
    np.random.seed(42)  # For reproducibility
    x, y, y_true = generate_noisy_line_data(n_points=50, slope=2.0, intercept=1.0, noise_level=0.5)
    print(f"Generated {len(x)} data points")
    
    # Fit line using Gauss-Newton method
    print("\nFitting line using Gauss-Newton method...")
    params, history = gauss_newton_line_fitting(x, y, max_iterations=100, tolerance=1e-6)
    
    # Plot results
    print("\nPlotting results...")
    plot_results(x, y, y_true, params, history)


if __name__ == "__main__":
    main()
