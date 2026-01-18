"""
AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
Semester 2, 2025-2026
Week 2: Fundamentals of Programming for AI

Copyright (c) 2025-2026
Department of Aeronautical and Aviation Engineering (AAE)
The Hong Kong Polytechnic University

Lecturer: Dr. Weisong Wen
Email: welson.wen@polyu.edu.hk
Address: R820, PolyU

This code is provided for educational purposes as part of the AAE4011 course.
All rights reserved.

Logistic Regression Function Plotting
This example demonstrates the logistic regression (sigmoid) function and its visualization.

Note: This code can also be run on Google Colab.
Google Colab: https://colab.research.google.com/
"""

import numpy as np
import matplotlib.pyplot as plt


def sigmoid_function(z):
    """
    Compute the sigmoid (logistic) function.
    
    The sigmoid function: σ(z) = 1 / (1 + e^(-z))
    
    Parameters:
    -----------
    z : numpy array or float
        Input values
    
    Returns:
    --------
    sigmoid : numpy array or float
        Sigmoid function values
    """
    return 1 / (1 + np.exp(-z))


def generate_sample_data(n_samples=100, seed=42):
    """
    Generate sample binary classification data for demonstration.
    
    Parameters:
    -----------
    n_samples : int
        Number of data points to generate
    seed : int
        Random seed for reproducibility
    
    Returns:
    --------
    x : numpy array
        Feature values
    y : numpy array
        Binary class labels (0 or 1)
    """
    np.random.seed(seed)
    x = np.linspace(-10, 10, n_samples)
    # Generate labels based on a logistic function with some noise
    z = -0.5 * x + 2
    p = sigmoid_function(z)
    y = (p > np.random.random(n_samples)).astype(int)
    return x, y


def plot_sigmoid_function():
    """
    Plot the basic sigmoid (logistic) function.
    """
    z = np.linspace(-10, 10, 1000)
    sigmoid = sigmoid_function(z)
    
    plt.figure(figsize=(10, 6))
    plt.plot(z, sigmoid, 'b-', linewidth=3, label='Sigmoid Function: σ(z) = 1/(1+e^(-z))')
    plt.axhline(y=0.5, color='r', linestyle='--', linewidth=1, label='Decision Threshold (0.5)')
    plt.axvline(x=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    plt.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    
    plt.xlabel('z', fontsize=14)
    plt.ylabel('σ(z)', fontsize=14)
    plt.title('Logistic (Sigmoid) Function', fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=12)
    plt.xlim(-10, 10)
    plt.ylim(-0.1, 1.1)
    
    # Add annotations
    plt.text(0, 0.5, '(0, 0.5)', fontsize=10, ha='center', va='bottom')
    plt.text(-5, 0.02, 'z → -∞, σ(z) → 0', fontsize=10, ha='center')
    plt.text(5, 0.98, 'z → +∞, σ(z) → 1', fontsize=10, ha='center')
    
    plt.tight_layout()
    plt.show()


def plot_logistic_regression_curve(x_data, y_data, coefficients=None):
    """
    Plot logistic regression curve fitted to data.
    
    Parameters:
    -----------
    x_data : numpy array
        Feature values
    y_data : numpy array
        Binary class labels
    coefficients : tuple or None
        (slope, intercept) for the linear combination z = slope*x + intercept
        If None, estimates from data using simple method
    """
    if coefficients is None:
        # Simple estimation (for demonstration - in practice use sklearn or proper optimization)
        # Fit a simple linear model first
        from sklearn.linear_model import LogisticRegression
        X = x_data.reshape(-1, 1)
        lr = LogisticRegression()
        lr.fit(X, y_data)
        slope = lr.coef_[0][0]
        intercept = lr.intercept_[0]
    else:
        slope, intercept = coefficients
    
    # Generate smooth curve
    x_curve = np.linspace(x_data.min(), x_data.max(), 1000)
    z_curve = slope * x_curve + intercept
    y_curve = sigmoid_function(z_curve)
    
    # Plot
    plt.figure(figsize=(12, 7))
    
    # Plot data points
    plt.scatter(x_data[y_data == 0], y_data[y_data == 0], 
                color='blue', marker='o', s=50, alpha=0.6, label='Class 0', edgecolors='black')
    plt.scatter(x_data[y_data == 1], y_data[y_data == 1], 
                color='red', marker='s', s=50, alpha=0.6, label='Class 1', edgecolors='black')
    
    # Plot logistic regression curve
    plt.plot(x_curve, y_curve, 'g-', linewidth=3, label=f'Logistic Regression: σ({slope:.3f}*x + {intercept:.3f})')
    
    # Decision boundary
    plt.axhline(y=0.5, color='orange', linestyle='--', linewidth=2, label='Decision Boundary (p=0.5)')
    
    plt.xlabel('Feature (x)', fontsize=14)
    plt.ylabel('Probability P(y=1|x)', fontsize=14)
    plt.title('Logistic Regression Curve', fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=11, loc='best')
    plt.ylim(-0.1, 1.1)
    
    plt.tight_layout()
    plt.show()


def plot_multiple_sigmoid_functions():
    """
    Plot sigmoid functions with different parameters to show the effect of slope and intercept.
    """
    z = np.linspace(-10, 10, 1000)
    
    # Different parameter combinations
    parameters = [
        {'slope': 1, 'intercept': 0, 'label': 'z = x (standard)', 'color': 'blue'},
        {'slope': 2, 'intercept': 0, 'label': 'z = 2x (steep)', 'color': 'green'},
        {'slope': 0.5, 'intercept': 0, 'label': 'z = 0.5x (gradual)', 'color': 'orange'},
        {'slope': 1, 'intercept': -2, 'label': 'z = x - 2 (shifted left)', 'color': 'red'},
        {'slope': 1, 'intercept': 2, 'label': 'z = x + 2 (shifted right)', 'color': 'purple'},
    ]
    
    plt.figure(figsize=(12, 7))
    
    for param in parameters:
        z_transformed = param['slope'] * z + param['intercept']
        sigmoid = sigmoid_function(z_transformed)
        plt.plot(z, sigmoid, linewidth=2.5, label=param['label'], color=param['color'])
    
    plt.axhline(y=0.5, color='black', linestyle='--', linewidth=1, alpha=0.5)
    plt.xlabel('x', fontsize=14)
    plt.ylabel('σ(z) where z = slope*x + intercept', fontsize=14)
    plt.title('Sigmoid Functions with Different Parameters', fontsize=16, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    plt.xlim(-10, 10)
    plt.ylim(-0.1, 1.1)
    
    plt.tight_layout()
    plt.show()


def plot_logistic_function_characteristics():
    """
    Plot showing key characteristics of the logistic function.
    """
    z = np.linspace(-10, 10, 1000)
    sigmoid = sigmoid_function(z)
    
    # Derivative of sigmoid (for showing the rate of change)
    sigmoid_derivative = sigmoid * (1 - sigmoid)
    
    fig, axes = plt.subplots(2, 1, figsize=(10, 10))
    
    # Plot 1: Sigmoid function
    ax1 = axes[0]
    ax1.plot(z, sigmoid, 'b-', linewidth=3, label='σ(z) = 1/(1+e^(-z))')
    ax1.axhline(y=0.5, color='r', linestyle='--', linewidth=1.5, label='Decision threshold (0.5)')
    ax1.axvline(x=0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
    ax1.fill_between(z, 0, 0.5, where=(sigmoid < 0.5), alpha=0.2, color='blue', label='Class 0 region')
    ax1.fill_between(z, 0.5, 1, where=(sigmoid >= 0.5), alpha=0.2, color='red', label='Class 1 region')
    
    ax1.set_xlabel('z', fontsize=12)
    ax1.set_ylabel('σ(z)', fontsize=12)
    ax1.set_title('Logistic (Sigmoid) Function', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)
    ax1.set_xlim(-10, 10)
    ax1.set_ylim(-0.1, 1.1)
    
    # Plot 2: Derivative (showing where function changes most rapidly)
    ax2 = axes[1]
    ax2.plot(z, sigmoid_derivative, 'g-', linewidth=3, label="σ'(z) = σ(z)(1-σ(z))")
    ax2.axvline(x=0, color='r', linestyle='--', linewidth=1.5, label='Maximum slope at z=0')
    
    ax2.set_xlabel('z', fontsize=12)
    ax2.set_ylabel("σ'(z)", fontsize=12)
    ax2.set_title('Derivative of Sigmoid Function (Rate of Change)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)
    ax2.set_xlim(-10, 10)
    
    plt.tight_layout()
    plt.show()


def main():
    """
    Main function to demonstrate logistic regression function plotting.
    """
    print("Logistic Regression Function Plotting")
    print("="*60)
    
    # Install sklearn if needed (for logistic regression fitting)
    try:
        from sklearn.linear_model import LogisticRegression
        has_sklearn = True
    except ImportError:
        print("\nNote: scikit-learn not found. Some plots will use simplified fitting.")
        print("Install with: pip install scikit-learn")
        has_sklearn = False
    
    print("\n1. Plotting basic sigmoid function...")
    plot_sigmoid_function()
    
    print("\n2. Plotting sigmoid functions with different parameters...")
    plot_multiple_sigmoid_functions()
    
    print("\n3. Plotting logistic function characteristics...")
    plot_logistic_function_characteristics()
    
    if has_sklearn:
        print("\n4. Plotting logistic regression curve with sample data...")
        x_data, y_data = generate_sample_data(n_samples=100)
        plot_logistic_regression_curve(x_data, y_data)
    
    print("\n" + "="*60)
    print("Plotting complete!")
    print("\nKey Properties of Logistic Function:")
    print("  - Domain: (-∞, +∞)")
    print("  - Range: (0, 1)")
    print("  - S-shaped (sigmoid) curve")
    print("  - Output can be interpreted as probability")
    print("  - Decision threshold typically at 0.5")
    print("="*60)


if __name__ == "__main__":
    main()
