import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error, r2_score, accuracy_score, confusion_matrix, classification_report
import os

# Create figures directory if it doesn't exist
if not os.path.exists('figures'):
    os.makedirs('figures')

def load_and_preprocess_data():
    """
    Load the GPS NLOS dataset and perform preprocessing
    
    Returns:
    tuple: (X_train, X_test, y_train, y_test, X.columns)
    """
    print("Loading and preprocessing data...")
    
    # Load the dataset
    data = pd.read_csv('gps_nlos_dataset.csv')
    
    # Display basic statistics
    print("Dataset shape:", data.shape)
    print("\nClass distribution:")
    print(data['NLOS_Status'].value_counts())
    print("\nFeature statistics:")
    print(data.describe())
    
    # Split features and target
    if 'Environment' in data.columns:
        X = data[['SNR', 'Constellation', 'Elevation', 'Azimuth', 'Environment']]
        # Convert Environment to one-hot encoding
        X = pd.get_dummies(X, columns=['Environment', 'Constellation'], drop_first=True)
    else:
        X = data[['SNR', 'Constellation', 'Elevation', 'Azimuth']]
        # Convert Constellation to one-hot encoding
        X = pd.get_dummies(X, columns=['Constellation'], drop_first=True)
    
    y = data['NLOS_Status']
    
    # Split into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.3, random_state=42
    )
    
    print(f"Training set: {X_train.shape[0]} samples")
    print(f"Testing set: {X_test.shape[0]} samples")
    
    return X_train, X_test, y_train, y_test, X.columns

def implement_linear_regression(X_train, X_test, y_train, y_test):
    """
    Implement linear regression model and make predictions
    
    Parameters:
    X_train, X_test, y_train, y_test: Training and testing sets
    
    Returns:
    tuple: (lin_reg, y_pred_train, y_pred_test, y_pred_binary)
    """
    print("\nImplementing linear regression...")
    
    # Create and train the model
    lin_reg = LinearRegression()
    lin_reg.fit(X_train, y_train)
    
    # Make predictions
    y_pred_train = lin_reg.predict(X_train)
    y_pred_test = lin_reg.predict(X_test)
    
    # Convert predictions to binary (0/1)
    y_pred_binary = (y_pred_test > 0.5).astype(int)
    
    return lin_reg, y_pred_train, y_pred_test, y_pred_binary

def evaluate_model(y_test, y_pred_test, y_pred_binary):
    """
    Evaluate the linear regression model
    
    Parameters:
    y_test: True values
    y_pred_test: Predicted continuous values
    y_pred_binary: Predicted binary values
    """
    print("\nEvaluating model performance...")
    
    # Calculate performance metrics
    mse = mean_squared_error(y_test, y_pred_test)
    r2 = r2_score(y_test, y_pred_test)
    accuracy = accuracy_score(y_test, y_pred_binary)
    conf_matrix = confusion_matrix(y_test, y_pred_binary)
    
    print(f"Mean Squared Error: {mse:.4f}")
    print(f"R² Score: {r2:.4f}")
    print(f"Accuracy: {accuracy:.4f}")
    print("\nConfusion Matrix:")
    print(conf_matrix)
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred_binary))
    
    # Visualize confusion matrix
    plt.figure(figsize=(8, 6))
    sns.heatmap(conf_matrix, annot=True, fmt='d', cmap='Blues',
               xticklabels=['LOS (0)', 'NLOS (1)'],
               yticklabels=['LOS (0)', 'NLOS (1)'])
    plt.title('Confusion Matrix')
    plt.ylabel('True Label')
    plt.xlabel('Predicted Label')
    plt.tight_layout()
    plt.savefig('figures/linear_regression_confusion_matrix.png')
    
    # Plot predicted probabilities vs true values
    plt.figure(figsize=(10, 6))
    
    # Create jittered actual values for better visualization
    jittered_true = y_test + np.random.normal(0, 0.05, len(y_test))
    jittered_true = np.clip(jittered_true, -0.1, 1.1)  # Keep within reasonable bounds
    
    plt.scatter(jittered_true, y_pred_test, alpha=0.5)
    plt.axhline(y=0.5, color='r', linestyle='--', label='Decision Boundary')
    plt.axvline(x=0.5, color='g', linestyle='--')
    
    # Add quadrants labels
    plt.text(0.2, 0.8, 'False Positives', ha='center')
    plt.text(0.8, 0.2, 'False Negatives', ha='center')
    plt.text(0.2, 0.2, 'True Negatives', ha='center')
    plt.text(0.8, 0.8, 'True Positives', ha='center')
    
    plt.grid(True, alpha=0.3)
    plt.xlabel('True Values (jittered)')
    plt.ylabel('Predicted Values')
    plt.title('Linear Regression Predictions vs True Values')
    plt.tight_layout()
    plt.savefig('figures/linear_regression_predictions.png')
    
    return mse, r2, accuracy

def analyze_coefficients(lin_reg, feature_names):
    """
    Analyze and visualize the coefficients of the linear regression model
    
    Parameters:
    lin_reg: Trained linear regression model
    feature_names: Names of the features
    
    Returns:
    DataFrame: Sorted coefficients
    """
    print("\nAnalyzing feature coefficients...")
    
    # Create DataFrame with coefficients
    coefficients = pd.DataFrame({
        'Feature': feature_names,
        'Coefficient': lin_reg.coef_
    })
    
    # Sort by absolute coefficient value
    sorted_coefficients = coefficients.reindex(
        coefficients['Coefficient'].abs().sort_values(ascending=False).index
    )
    
    print("Intercept:", lin_reg.intercept_)
    print("\nCoefficients (sorted by absolute value):")
    print(sorted_coefficients)
    
    # Visualize coefficients
    plt.figure(figsize=(12, 8))
    
    # Color bars based on coefficient sign
    colors = ['red' if c < 0 else 'blue' for c in sorted_coefficients['Coefficient']]
    
    # Plot horizontal bar chart
    bars = plt.barh(sorted_coefficients['Feature'], sorted_coefficients['Coefficient'], color=colors)
    
    # Add coefficient values as text
    for i, bar in enumerate(bars):
        value = sorted_coefficients['Coefficient'].iloc[i]
        text_x = max(0.01, min(-0.01, value * 0.2))  # Offset to place text
        text_color = 'black' if abs(value) > 0.2 else 'white'
        plt.text(text_x, i, f'{value:.4f}', 
                 va='center', ha='left' if value < 0 else 'right',
                 color=text_color, fontweight='bold')
    
    plt.axvline(x=0, color='black', linestyle='-', alpha=0.7)
    plt.xlabel('Coefficient Value')
    plt.ylabel('Feature')
    plt.title('Linear Regression Coefficients')
    plt.grid(axis='x', alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/linear_regression_coefficients.png')
    
    return sorted_coefficients

def interpret_results(accuracy, r2, sorted_coefficients):
    """
    Interpret the results of the linear regression model
    
    Parameters:
    accuracy: Model accuracy
    r2: R² score
    sorted_coefficients: Sorted coefficients DataFrame
    """
    print("\nInterpreting results...")
    
    # Get top positive and negative features
    top_positive = sorted_coefficients[sorted_coefficients['Coefficient'] > 0].head(3)
    top_negative = sorted_coefficients[sorted_coefficients['Coefficient'] < 0].head(3)
    
    print("Linear Regression Interpretation:")
    print(f"- Linear regression achieved an accuracy of {accuracy*100:.2f}%")
    print(f"- The model explains {r2*100:.2f}% of the variance in the data")
    
    print("\n- Features most predictive of NLOS (positive coefficients):")
    for _, row in top_positive.iterrows():
        print(f"  * {row['Feature']}: {row['Coefficient']:.4f}")
    
    print("\n- Features most predictive of LOS (negative coefficients):")
    for _, row in top_negative.iterrows():
        print(f"  * {row['Feature']}: {row['Coefficient']:.4f}")
    
    # Create a summary figure
    plt.figure(figsize=(12, 8))
    
    # Set up 2x2 subplots
    plt.subplot(2, 2, 1)
    plt.bar(['Accuracy'], [accuracy])
    plt.ylim(0, 1)
    plt.title('Model Accuracy')
    plt.grid(axis='y', alpha=0.3)
    
    plt.subplot(2, 2, 2)
    plt.bar(['R² Score'], [r2])
    plt.ylim(0, 1)
    plt.title('R² Score (Variance Explained)')
    plt.grid(axis='y', alpha=0.3)
    
    plt.subplot(2, 2, 3)
    plt.barh(top_positive['Feature'], top_positive['Coefficient'], color='red')
    plt.title('Top Features Predicting NLOS')
    plt.grid(axis='x', alpha=0.3)
    
    plt.subplot(2, 2, 4)
    plt.barh(top_negative['Feature'], top_negative['Coefficient'].abs(), color='blue')
    plt.title('Top Features Predicting LOS (abs values)')
    plt.grid(axis='x', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('figures/linear_regression_summary.png')

def main():
    """
    Main function to run the linear regression implementation
    """
    print("Exercise 2: Implementing Linear Regression for NLOS Detection\n")
    
    # Load and preprocess data
    X_train, X_test, y_train, y_test, feature_names = load_and_preprocess_data()
    
    # Implement linear regression
    lin_reg, y_pred_train, y_pred_test, y_pred_binary = implement_linear_regression(
        X_train, X_test, y_train, y_test
    )
    
    # Evaluate the model
    mse, r2, accuracy = evaluate_model(y_test, y_pred_test, y_pred_binary)
    
    # Analyze coefficients
    sorted_coefficients = analyze_coefficients(lin_reg, feature_names)
    
    # Interpret results
    interpret_results(accuracy, r2, sorted_coefficients)
    
    print("\nExercise 2 completed successfully!")
    print("Visualizations saved in the 'figures' directory.")

if __name__ == "__main__":
    main()