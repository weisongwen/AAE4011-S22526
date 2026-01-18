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

Interactive Logistic Regression UI
This example provides an interactive interface to adjust noise levels and 
logistic function parameters with real-time visualization.

Note: This code can also be run on Google Colab.
Google Colab: https://colab.research.google.com/
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button


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


def generate_noisy_data(x, slope, intercept, noise_level, seed=None):
    """
    Generate noisy binary classification data.
    
    Parameters:
    -----------
    x : numpy array
        Feature values
    slope : float
        Slope parameter for logistic function
    intercept : float
        Intercept parameter for logistic function
    noise_level : float
        Standard deviation of noise
    seed : int, optional
        Random seed for reproducibility
    
    Returns:
    --------
    y : numpy array
        Binary class labels (0 or 1)
    """
    if seed is not None:
        np.random.seed(seed)
    
    # Compute probability using logistic function
    z = slope * x + intercept
    p = sigmoid_function(z)
    
    # Add noise and generate binary labels
    noise = np.random.normal(0, noise_level, len(x))
    p_noisy = np.clip(p + noise, 0, 1)  # Clip to [0, 1] range
    y = (p_noisy > 0.5).astype(int)
    
    return y


def interactive_logistic_regression():
    """
    Create an interactive interface for adjusting logistic regression parameters.
    """
    # Initial parameters
    initial_slope = 1.0
    initial_intercept = 0.0
    initial_noise = 0.1
    n_points = 100
    
    # Generate initial data
    x = np.linspace(-10, 10, n_points)
    
    # Create figure with proper spacing for sliders
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111)
    fig.subplots_adjust(bottom=0.35, left=0.1, right=0.95, top=0.95)
    
    # Initial data generation
    y = generate_noisy_data(x, initial_slope, initial_intercept, initial_noise, seed=42)
    
    # Generate smooth curve for initial plot
    x_curve = np.linspace(x.min(), x.max(), 1000)
    z_curve = initial_slope * x_curve + initial_intercept
    y_curve = sigmoid_function(z_curve)
    
    # Plot initial data points
    scatter0 = ax.scatter(x[y == 0], y[y == 0], 
                         color='blue', marker='o', s=60, alpha=0.7, 
                         label='Class 0', edgecolors='black', linewidths=1)
    scatter1 = ax.scatter(x[y == 1], y[y == 1], 
                         color='red', marker='s', s=60, alpha=0.7, 
                         label='Class 1', edgecolors='black', linewidths=1)
    
    # Plot initial logistic curve
    line_curve, = ax.plot(x_curve, y_curve, 'g-', linewidth=3, 
                         label=f'Logistic: σ({initial_slope:.2f}*x + {initial_intercept:.2f})')
    
    # Decision boundary (horizontal)
    hline = ax.axhline(y=0.5, color='orange', linestyle='--', linewidth=2, 
                      label='Decision Boundary (p=0.5)')
    
    # Decision point (vertical) - will be created if needed (use list for mutability)
    vline_container = [None]
    
    # Set up plot labels and formatting
    ax.set_xlabel('Feature (x)', fontsize=12)
    ax.set_ylabel('Probability P(y=1|x)', fontsize=12)
    ax.set_title(f'Interactive Logistic Regression (Noise: {initial_noise:.2f})', 
                fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=10, loc='best')
    ax.set_ylim(-0.1, 1.1)
    ax.set_xlim(x.min(), x.max())
    
    # Create sliders with better spacing
    ax_slope = fig.add_axes([0.2, 0.25, 0.6, 0.03])
    ax_intercept = fig.add_axes([0.2, 0.20, 0.6, 0.03])
    ax_noise = fig.add_axes([0.2, 0.15, 0.6, 0.03])
    
    slider_slope = Slider(ax_slope, 'Slope', -3.0, 3.0, 
                          valinit=initial_slope, valstep=0.1)
    slider_intercept = Slider(ax_intercept, 'Intercept', -5.0, 5.0, 
                               valinit=initial_intercept, valstep=0.1)
    slider_noise = Slider(ax_noise, 'Noise Level', 0.0, 1.0, 
                          valinit=initial_noise, valstep=0.01)
    
    # Store scatter plots in a mutable list for update function
    scatter_plots = [scatter0, scatter1]
    
    # Update function
    def update(val):
        slope = slider_slope.val
        intercept = slider_intercept.val
        noise = slider_noise.val
        
        # Regenerate data with new parameters
        y_new = generate_noisy_data(x, slope, intercept, noise, seed=42)
        
        # Update scatter plots by removing and recreating
        scatter_plots[0].remove()
        scatter_plots[1].remove()
        scatter_plots[0] = ax.scatter(x[y_new == 0], y_new[y_new == 0], 
                                     color='blue', marker='o', s=60, alpha=0.7, 
                                     label='Class 0', edgecolors='black', linewidths=1)
        scatter_plots[1] = ax.scatter(x[y_new == 1], y_new[y_new == 1], 
                                     color='red', marker='s', s=60, alpha=0.7, 
                                     label='Class 1', edgecolors='black', linewidths=1)
        
        # Update logistic curve
        z_curve_new = slope * x_curve + intercept
        y_curve_new = sigmoid_function(z_curve_new)
        line_curve.set_ydata(y_curve_new)
        line_curve.set_label(f'Logistic: σ({slope:.2f}*x + {intercept:.2f})')
        
        # Update decision boundary vertical line
        if vline_container[0] is not None:
            vline_container[0].remove()
        vline_container[0] = None
        
        if slope != 0:
            decision_x = -intercept / slope
            if x.min() <= decision_x <= x.max():
                vline_container[0] = ax.axvline(x=decision_x, color='orange', linestyle='--', 
                                               linewidth=1, alpha=0.5)
        
        # Update title
        ax.set_title(f'Interactive Logistic Regression (Noise: {noise:.2f})', 
                    fontsize=14, fontweight='bold')
        
        # Update legend
        ax.legend(fontsize=10, loc='best')
        
        # Redraw
        fig.canvas.draw_idle()
    
    # Connect sliders to update function
    slider_slope.on_changed(update)
    slider_intercept.on_changed(update)
    slider_noise.on_changed(update)
    
    # Add reset button
    resetax = fig.add_axes([0.85, 0.22, 0.1, 0.04])
    button_reset = Button(resetax, 'Reset', color='lightgray', hovercolor='0.975')
    
    def reset(event):
        slider_slope.reset()
        slider_intercept.reset()
        slider_noise.reset()
    
    button_reset.on_clicked(reset)
    
    plt.show()
    return fig, slider_slope, slider_intercept, slider_noise


def simple_interactive_demo():
    """
    A simpler version that works better in non-interactive environments.
    Shows multiple plots with different parameter combinations.
    """
    x = np.linspace(-10, 10, 100)
    
    # Different parameter combinations to demonstrate
    parameter_sets = [
        {'slope': 0.5, 'intercept': 0, 'noise': 0.1, 'title': 'Gradual (low slope, low noise)'},
        {'slope': 1.0, 'intercept': 0, 'noise': 0.1, 'title': 'Standard (medium slope, low noise)'},
        {'slope': 2.0, 'intercept': 0, 'noise': 0.1, 'title': 'Steep (high slope, low noise)'},
        {'slope': 1.0, 'intercept': 0, 'noise': 0.3, 'title': 'Noisy (medium slope, high noise)'},
        {'slope': 1.0, 'intercept': -2, 'noise': 0.1, 'title': 'Shifted Left (intercept = -2)'},
        {'slope': 1.0, 'intercept': 2, 'noise': 0.1, 'title': 'Shifted Right (intercept = 2)'},
    ]
    
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()
    
    for idx, params in enumerate(parameter_sets):
        ax = axes[idx]
        
        # Generate data
        y = generate_noisy_data(x, params['slope'], params['intercept'], 
                                params['noise'], seed=42)
        
        # Plot data points
        ax.scatter(x[y == 0], y[y == 0], color='blue', marker='o', s=30, 
                  alpha=0.6, label='Class 0', edgecolors='black')
        ax.scatter(x[y == 1], y[y == 1], color='red', marker='s', s=30, 
                  alpha=0.6, label='Class 1', edgecolors='black')
        
        # Plot curve
        x_curve = np.linspace(x.min(), x.max(), 1000)
        z_curve = params['slope'] * x_curve + params['intercept']
        y_curve = sigmoid_function(z_curve)
        ax.plot(x_curve, y_curve, 'g-', linewidth=2, 
               label=f"σ({params['slope']:.1f}*x + {params['intercept']:.1f})")
        
        ax.axhline(y=0.5, color='orange', linestyle='--', linewidth=1.5)
        ax.set_title(params['title'], fontsize=11, fontweight='bold')
        ax.set_xlabel('x', fontsize=10)
        ax.set_ylabel('P(y=1|x)', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.1, 1.1)
        if idx == 0:
            ax.legend(fontsize=8)
    
    plt.suptitle('Logistic Regression with Different Parameters and Noise Levels', 
                 fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    plt.show()


def interactive_logistic_regression_colab():
    """
    Create an interactive interface using ipywidgets (works better in Colab/Jupyter).
    Usage in Colab: Just call this function and it will display the interactive widgets.
    """
    try:
        import ipywidgets as widgets
    except ImportError:
        print("ipywidgets not found. Install with: !pip install ipywidgets")
        return None
    
    # Initial parameters
    n_points = 100
    x = np.linspace(-10, 10, n_points)
    
    def update_plot(slope=1.0, intercept=0.0, noise=0.1):
        """Update the plot with new parameters."""
        # Generate data
        y = generate_noisy_data(x, slope, intercept, noise, seed=42)
        
        # Create plot
        fig, ax = plt.subplots(figsize=(12, 7))
        
        # Plot data points
        ax.scatter(x[y == 0], y[y == 0], 
                  color='blue', marker='o', s=60, alpha=0.7, 
                  label='Class 0', edgecolors='black', linewidths=1)
        ax.scatter(x[y == 1], y[y == 1], 
                  color='red', marker='s', s=60, alpha=0.7, 
                  label='Class 1', edgecolors='black', linewidths=1)
        
        # Plot logistic curve
        x_curve = np.linspace(x.min(), x.max(), 1000)
        z_curve = slope * x_curve + intercept
        y_curve = sigmoid_function(z_curve)
        ax.plot(x_curve, y_curve, 'g-', linewidth=3, 
               label=f'Logistic: σ({slope:.2f}*x + {intercept:.2f})')
        
        # Decision boundary
        ax.axhline(y=0.5, color='orange', linestyle='--', linewidth=2, 
                  label='Decision Boundary (p=0.5)')
        
        # Vertical decision line
        if slope != 0:
            decision_x = -intercept / slope
            if x.min() <= decision_x <= x.max():
                ax.axvline(x=decision_x, color='orange', linestyle='--', 
                         linewidth=1, alpha=0.5)
        
        ax.set_xlabel('Feature (x)', fontsize=12)
        ax.set_ylabel('Probability P(y=1|x)', fontsize=12)
        ax.set_title(f'Interactive Logistic Regression (Noise: {noise:.2f})', 
                    fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10, loc='best')
        ax.set_ylim(-0.1, 1.1)
        
        plt.tight_layout()
        plt.show()
    
    # Create sliders
    slope_slider = widgets.FloatSlider(
        value=1.0,
        min=-3.0,
        max=3.0,
        step=0.1,
        description='Slope:',
        style={'description_width': 'initial'},
        layout=widgets.Layout(width='600px')
    )
    
    intercept_slider = widgets.FloatSlider(
        value=0.0,
        min=-5.0,
        max=5.0,
        step=0.1,
        description='Intercept:',
        style={'description_width': 'initial'},
        layout=widgets.Layout(width='600px')
    )
    
    noise_slider = widgets.FloatSlider(
        value=0.1,
        min=0.0,
        max=1.0,
        step=0.01,
        description='Noise Level:',
        style={'description_width': 'initial'},
        layout=widgets.Layout(width='600px')
    )
    
    # Create reset button
    reset_button = widgets.Button(
        description='Reset',
        button_style='info',
        layout=widgets.Layout(width='100px', height='30px')
    )
    
    def on_reset(b):
        slope_slider.value = 1.0
        intercept_slider.value = 0.0
        noise_slider.value = 0.1
    
    reset_button.on_click(on_reset)
    
    # Create interactive widget - this automatically links sliders to update function
    interactive_plot = widgets.interactive(
        update_plot, 
        slope=slope_slider, 
        intercept=intercept_slider, 
        noise=noise_slider
    )
    
    # Add reset button to the controls
    controls = widgets.VBox([
        widgets.HBox([slope_slider]),
        widgets.HBox([intercept_slider]),
        widgets.HBox([noise_slider, reset_button]),
    ])
    
    # Combine controls with the output from interactive widget
    final_widget = widgets.VBox([controls, interactive_plot.children[-1]])
    
    return final_widget


def main():
    """
    Main function to run the interactive logistic regression UI.
    """
    print("Interactive Logistic Regression UI")
    print("="*60)
    print("\nThis interface allows you to adjust:")
    print("  - Slope: Controls the steepness of the logistic curve")
    print("  - Intercept: Shifts the curve left or right")
    print("  - Noise Level: Controls the randomness in the data")
    print("\nAdjust the sliders to see real-time updates!")
    print("="*60)
    
    # Try ipywidgets first (for Colab/Jupyter)
    try:
        import ipywidgets
        print("\nDetected Jupyter/Colab environment. Using ipywidgets...")
        print("Note: In Colab, you may need to run: !pip install ipywidgets")
        controls = interactive_logistic_regression_colab()
        if controls is not None:
            from IPython.display import display
            display(controls)
            return
    except (ImportError, NameError):
        pass
    
    # Try matplotlib widgets (for local Python)
    try:
        print("\nCreating interactive plot with matplotlib sliders...")
        print("Note: Close the plot window to exit.")
        interactive_logistic_regression()
    except Exception as e:
        print(f"\nInteractive mode not available: {e}")
        print("Falling back to static demonstration plots...")
        simple_interactive_demo()
    
    print("\n" + "="*60)
    print("Demonstration complete!")
    print("\nParameter Effects:")
    print("  - Higher slope → Steeper transition")
    print("  - Positive intercept → Curve shifts right")
    print("  - Negative intercept → Curve shifts left")
    print("  - Higher noise → More scattered data points")
    print("="*60)


if __name__ == "__main__":
    main()
