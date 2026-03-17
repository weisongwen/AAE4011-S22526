#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Logistic Regression vs Multi-Layer Perceptron (MLP) Comparison

This script demonstrates the difference between a linear classifier (Logistic
Regression) and a nonlinear classifier (MLP) using synthetic concentric-circle
data, and visualises their decision boundaries.

Author: Dr. Weisong Wen
Department of Aeronautical and Aviation Engineering
The Hong Kong Polytechnic University
"""

import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import accuracy_score, classification_report

# Set random seed for reproducibility
np.random.seed(42)

# Generate nonlinear data (concentric circles)
def generate_concentric_circles(n_samples=1000, noise=0.1):
    # First class: inner circle
    radius1 = 1
    theta1 = 2 * np.pi * np.random.rand(n_samples // 2)
    x1 = radius1 * np.cos(theta1)
    y1 = radius1 * np.sin(theta1)
    
    # Add noise
    x1 += noise * np.random.randn(n_samples // 2)
    y1 += noise * np.random.randn(n_samples // 2)
    
    # Second class: outer circle
    radius2 = 3
    theta2 = 2 * np.pi * np.random.rand(n_samples // 2)
    x2 = radius2 * np.cos(theta2)
    y2 = radius2 * np.sin(theta2)
    
    # Add noise
    x2 += noise * np.random.randn(n_samples // 2)
    y2 += noise * np.random.randn(n_samples // 2)
    
    # Combine data points
    X = np.vstack([np.column_stack((x1, y1)), np.column_stack((x2, y2))])
    y = np.hstack([np.zeros(n_samples // 2), np.ones(n_samples // 2)])
    
    return X, y

# Generate the data
X, y = generate_concentric_circles(n_samples=2000, noise=0.2)

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)

# Standardize features
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Train a logistic regression model
log_reg = LogisticRegression(max_iter=1000, random_state=42)
log_reg.fit(X_train_scaled, y_train)

# Train an MLP model
mlp = MLPClassifier(hidden_layer_sizes=(10, 10), max_iter=1000, activation='relu',
                    solver='adam', random_state=42, alpha=0.01)
mlp.fit(X_train_scaled, y_train)

# Make predictions
y_pred_log_reg = log_reg.predict(X_test_scaled)
y_pred_mlp = mlp.predict(X_test_scaled)

# Calculate accuracy
log_reg_accuracy = accuracy_score(y_test, y_pred_log_reg)
mlp_accuracy = accuracy_score(y_test, y_pred_mlp)

print("Logistic Regression Accuracy:", log_reg_accuracy)
print("MLP Accuracy:", mlp_accuracy)

print("\nLogistic Regression Classification Report:")
print(classification_report(y_test, y_pred_log_reg))

print("\nMLP Classification Report:")
print(classification_report(y_test, y_pred_mlp))

# Visualization function for decision boundaries
def plot_decision_boundaries(X, y, models, titles, accuracies):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Create mesh grid
    x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
    y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
    resolution = 0.02
    xx, yy = np.meshgrid(np.arange(x_min, x_max, resolution), 
                         np.arange(y_min, y_max, resolution))
    
    for i, (model, title, accuracy) in enumerate(zip(models, titles, accuracies)):
        mesh_points = np.c_[xx.ravel(), yy.ravel()]
        mesh_points_scaled = scaler.transform(mesh_points)
        Z = model.predict(mesh_points_scaled)
        Z = Z.reshape(xx.shape)
        
        axes[i].contourf(xx, yy, Z, alpha=0.4, cmap='viridis')
        scatter = axes[i].scatter(X[:, 0], X[:, 1], c=y, s=20, edgecolor='k', cmap='viridis')
        axes[i].set_title(f'{title} (Accuracy: {accuracy:.3f})')
        axes[i].set_xlabel('Feature 1')
        axes[i].set_ylabel('Feature 2')
        
    cbar = fig.colorbar(scatter, ax=axes)
    cbar.set_label('Class')
    
    plt.tight_layout()
    return fig

# Plot both models' decision boundaries
plot_decision_boundaries(
    X, y, 
    [log_reg, mlp], 
    ['Logistic Regression', 'Multi-Layer Perceptron'],
    [log_reg_accuracy, mlp_accuracy]
)

# Separate MLP plot
plt.figure(figsize=(10, 8))
plt.title(f'Multi-Layer Perceptron (Accuracy: {mlp_accuracy:.3f})')
x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
resolution = 0.02
xx, yy = np.meshgrid(np.arange(x_min, x_max, resolution), 
                     np.arange(y_min, y_max, resolution))
mesh_points = np.c_[xx.ravel(), yy.ravel()]
mesh_points_scaled = scaler.transform(mesh_points)
Z = mlp.predict(mesh_points_scaled)
Z = Z.reshape(xx.shape)
plt.contourf(xx, yy, Z, alpha=0.3, cmap='viridis')
plt.scatter(X[:, 0], X[:, 1], c=y, edgecolor='k', alpha=0.8)
plt.xlabel('Feature 1')
plt.ylabel('Feature 2')
plt.colorbar()

plt.show()
