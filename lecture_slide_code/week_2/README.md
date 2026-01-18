# Week 2: Fundamentals of Programming for AI

This folder contains example code for Week 2 of AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems.

## Overview

The code example demonstrates the **logistic regression (sigmoid) function** and provides various visualizations to help understand its properties and behavior. The logistic regression function is a fundamental component of binary classification in machine learning.

## Files Description

### `logic_regression.py`
**Logistic Regression Function Plotting**

This script provides comprehensive visualizations of the sigmoid (logistic) function, which is the core mathematical function used in logistic regression for binary classification.

**Features:**
- Sigmoid function implementation: σ(z) = 1/(1+e^(-z))
- Multiple visualization plots showing different aspects of the logistic function
- Sample data generation for demonstration
- Logistic regression curve fitting (with scikit-learn)

**Key Functions:**

1. **`sigmoid_function(z)`**
   - Computes the sigmoid function: σ(z) = 1/(1+e^(-z))
   - Maps any real number to the range (0, 1)
   - Output can be interpreted as probability

2. **`plot_sigmoid_function()`**
   - Plots the basic sigmoid curve
   - Shows decision threshold at 0.5
   - Demonstrates asymptotic behavior as z → ±∞

3. **`plot_multiple_sigmoid_functions()`**
   - Shows how different parameters (slope and intercept) affect the curve
   - Demonstrates steepness and shifting effects
   - Helps understand the relationship z = slope*x + intercept

4. **`plot_logistic_function_characteristics()`**
   - Shows both the sigmoid function and its derivative
   - Illustrates where the function changes most rapidly
   - Demonstrates the maximum slope at z=0

5. **`plot_logistic_regression_curve(x_data, y_data)`**
   - Fits logistic regression to sample data
   - Shows the fitted curve overlaid on data points
   - Demonstrates binary classification visualization

6. **`generate_sample_data()`**
   - Generates synthetic binary classification data
   - Useful for demonstration and testing

## Requirements

The code requires the following Python packages:

- `numpy` - For numerical computations
- `matplotlib` - For plotting and visualization
- `scikit-learn` (optional) - For logistic regression fitting

**Installation:**
```bash
pip install numpy matplotlib scikit-learn
```

## Usage

### Running the Complete Example

```bash
python logic_regression.py
```

This will generate multiple plots demonstrating different aspects of the logistic function.

### Running on Google Colab

The code is compatible with Google Colab. Simply:
1. Upload the file to Google Colab
2. Install required packages: `!pip install numpy matplotlib scikit-learn`
3. Run the script

### Using Individual Functions

You can also import and use individual functions:

```python
from logic_regression import sigmoid_function, plot_sigmoid_function

# Compute sigmoid values
z = np.linspace(-10, 10, 100)
probabilities = sigmoid_function(z)

# Plot the sigmoid function
plot_sigmoid_function()
```

## Understanding the Logistic Function

### Key Properties

1. **Mathematical Formula:**
   ```
   σ(z) = 1 / (1 + e^(-z))
   ```

2. **Domain and Range:**
   - **Domain:** (-∞, +∞) - accepts any real number
   - **Range:** (0, 1) - outputs values between 0 and 1

3. **Characteristics:**
   - **S-shaped curve** (sigmoid)
   - **Monotonic:** Always increasing
   - **Symmetric:** Around the point (0, 0.5)
   - **Asymptotic:** Approaches 0 as z → -∞, approaches 1 as z → +∞

4. **Decision Boundary:**
   - Typically at σ(z) = 0.5
   - Corresponds to z = 0
   - Class 0: σ(z) < 0.5
   - Class 1: σ(z) ≥ 0.5

5. **Derivative:**
   ```
   σ'(z) = σ(z) * (1 - σ(z))
   ```
   - Maximum at z = 0
   - Decreases as |z| increases

### Why Use Logistic Regression?

- **Probabilistic Interpretation:** Output represents probability P(y=1|x)
- **Non-linear Transformation:** Maps linear combination to (0, 1) range
- **Smooth and Differentiable:** Important for optimization algorithms
- **Interpretable:** Easy to understand decision boundaries

## Plot Outputs

The script generates the following plots:

1. **Basic Sigmoid Function**
   - Standard sigmoid curve with decision threshold

2. **Multiple Parameter Variations**
   - Shows effects of different slope and intercept values
   - Demonstrates how parameters control curve shape

3. **Function Characteristics**
   - Sigmoid function and its derivative
   - Shows rate of change at different points

4. **Logistic Regression Curve** (if scikit-learn is installed)
   - Fitted curve on sample binary classification data
   - Real-world application visualization

## Mathematical Background

### Logistic Regression Model

In logistic regression, we model the probability of a binary outcome:

```
P(y=1|x) = σ(w₀ + w₁x₁ + w₂x₂ + ... + wₙxₙ)
```

Where:
- `σ` is the sigmoid function
- `w₀` is the intercept (bias)
- `w₁, w₂, ..., wₙ` are the coefficients (weights)
- `x₁, x₂, ..., xₙ` are the features

### Linear Combination

The argument to the sigmoid function is a linear combination:
```
z = w₀ + w₁x₁ + w₂x₂ + ... + wₙxₙ
```

This linear combination is then transformed by the sigmoid function to produce a probability.

## Troubleshooting

**Issue: Import error for scikit-learn**
- **Solution:** Install with `pip install scikit-learn`
- Note: The code will still work without it, but the logistic regression curve plot will be skipped

**Issue: Matplotlib display issues**
- **Solution:** If using WSL or remote server, you may need to set backend: `matplotlib.use('Agg')`
- For GUI display, ensure you have a display server running

**Issue: Plot window not showing**
- **Solution:** Make sure you're running in an interactive environment
- In scripts, use `plt.show()` to display plots

## Applications in Unmanned Autonomous Systems

Logistic regression and the sigmoid function are fundamental in AI for autonomous systems:

- **Classification Tasks:** Binary decision making (e.g., obstacle detection)
- **Probability Estimation:** Confidence scores for predictions
- **Neural Networks:** Activation function in hidden layers
- **Sensor Fusion:** Combining multiple sensor inputs with confidence

## Course Information

- **Course**: AAE4011 - Artificial Intelligence for Unmanned Autonomous Systems
- **Semester**: 2, 2025-2026
- **Week**: Week 2 - Fundamentals of Programming for AI
- **Lecturer**: Dr. Weisong Wen
- **Email**: welson.wen@polyu.edu.hk
- **Department**: Aeronautical and Aviation Engineering (AAE), The Hong Kong Polytechnic University

## Additional Resources

- **NumPy Documentation**: https://numpy.org/doc/
- **Matplotlib Documentation**: https://matplotlib.org/stable/
- **Scikit-learn Logistic Regression**: https://scikit-learn.org/stable/modules/linear_model.html#logistic-regression

## License

Copyright (c) 2025-2026  
This code is provided for educational purposes as part of the AAE4011 course.  
All rights reserved.
