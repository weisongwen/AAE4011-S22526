import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (accuracy_score, confusion_matrix, classification_report,
                            roc_curve, roc_auc_score, precision_recall_curve, auc)
import os

# Create figures directory if it doesn't exist
if not os.path.exists('figures'):
    os.makedirs('figures')

def load_and_preprocess_data():
    """
    Load the GPS NLOS dataset and perform preprocessing
    
    Returns:
    tuple: (X_train, X_test, y_train, y_test, X.columns, original_X)
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
    
    # Store original X for later use
    original_X = X.copy()
    
    y = data['NLOS_Status']
    
    # Split into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.3, random_state=42
    )
    
    print(f"Training set: {X_train.shape[0]} samples")
    print(f"Testing set: {X_test.shape[0]} samples")
    
    return X_train, X_test, y_train, y_test, X.columns, original_X

def implement_logistic_regression(X_train, X_test, y_train, y_test):
    """
    Implement logistic regression model and make predictions
    
    Parameters:
    X_train, X_test, y_train, y_test: Training and testing sets
    
    Returns:
    tuple: (log_reg, y_pred, y_pred_proba)
    """
    print("\nImplementing logistic regression...")
    
    # Create and train the model
    log_reg = LogisticRegression(max_iter=1000, random_state=42)
    log_reg.fit(X_train, y_train)
    
    # Make predictions
    y_pred = log_reg.predict(X_test)
    y_pred_proba = log_reg.predict_proba(X_test)[:, 1]  # Probability of NLOS
    
    return log_reg, y_pred, y_pred_proba

def evaluate_model(y_test, y_pred, y_pred_proba):
    """
    Evaluate the logistic regression model
    
    Parameters:
    y_test: True values
    y_pred: Predicted class labels
    y_pred_proba: Predicted probabilities
    
    Returns:
    tuple: (accuracy, auc_score)
    """
    print("\nEvaluating model performance...")
    
    # Calculate performance metrics
    accuracy = accuracy_score(y_test, y_pred)
    conf_matrix = confusion_matrix(y_test, y_pred)
    auc_score = roc_auc_score(y_test, y_pred_proba)
    
    print(f"Accuracy: {accuracy:.4f}")
    print(f"AUC Score: {auc_score:.4f}")
    print("\nConfusion Matrix:")
    print(conf_matrix)
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred))
    
    # Visualize confusion matrix
    plt.figure(figsize=(8, 6))
    sns.heatmap(conf_matrix, annot=True, fmt='d', cmap='Blues',
               xticklabels=['LOS (0)', 'NLOS (1)'],
               yticklabels=['LOS (0)', 'NLOS (1)'])
    plt.title('Confusion Matrix - Logistic Regression')
    plt.ylabel('True Label')
    plt.xlabel('Predicted Label')
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_confusion_matrix.png')
    
    # Plot ROC curve
    fpr, tpr, thresholds = roc_curve(y_test, y_pred_proba)
    
    plt.figure(figsize=(10, 6))
    plt.plot(fpr, tpr, label=f'Logistic Regression (AUC = {auc_score:.3f})')
    plt.plot([0, 1], [0, 1], 'k--', label='Random Classifier')
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    plt.title('ROC Curve - Logistic Regression')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_roc.png')
    
    # Plot Precision-Recall curve
    precision, recall, _ = precision_recall_curve(y_test, y_pred_proba)
    pr_auc = auc(recall, precision)
    
    plt.figure(figsize=(10, 6))
    plt.plot(recall, precision, label=f'Logistic Regression (AUC = {pr_auc:.3f})')
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.title('Precision-Recall Curve - Logistic Regression')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_precision_recall.png')
    
    # Plot histogram of predicted probabilities
    plt.figure(figsize=(10, 6))
    plt.hist(y_pred_proba[y_test == 0], bins=20, alpha=0.5, label='LOS (Class 0)', color='blue')
    plt.hist(y_pred_proba[y_test == 1], bins=20, alpha=0.5, label='NLOS (Class 1)', color='red')
    plt.xlabel('Predicted Probability of NLOS')
    plt.ylabel('Count')
    plt.title('Distribution of Predicted Probabilities')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_probability_distribution.png')
    
    return accuracy, auc_score

def analyze_coefficients(log_reg, feature_names):
    """
    Analyze and visualize the coefficients of the logistic regression model
    
    Parameters:
    log_reg: Trained logistic regression model
    feature_names: Names of the features
    
    Returns:
    DataFrame: Sorted coefficients
    """
    print("\nAnalyzing feature coefficients...")
    
    # Create DataFrame with coefficients and odds ratios
    coefficients = pd.DataFrame({
        'Feature': feature_names,
        'Coefficient': log_reg.coef_[0],
        'Odds_Ratio': np.exp(log_reg.coef_[0])
    })
    
    # Sort by absolute coefficient value
    sorted_coefficients = coefficients.reindex(
        coefficients['Coefficient'].abs().sort_values(ascending=False).index
    )
    
    print("Intercept:", log_reg.intercept_[0])
    print("\nCoefficients and Odds Ratios (sorted by absolute coefficient value):")
    print(sorted_coefficients)
    
    # Visualize coefficients
    plt.figure(figsize=(12, 8))
    
    # Color bars based on coefficient sign
    colors = ['red' if c < 0 else 'blue' for c in sorted_coefficients['Coefficient']]
    
    plt.barh(sorted_coefficients['Feature'], sorted_coefficients['Coefficient'], color=colors)
    plt.axvline(x=0, color='black', linestyle='-', alpha=0.7)
    plt.xlabel('Coefficient Value')
    plt.ylabel('Feature')
    plt.title('Logistic Regression Coefficients')
    plt.grid(axis='x', alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_coefficients.png')
    
    # Visualize odds ratios
    plt.figure(figsize=(12, 8))
    
    # Only plot features with significant odds ratios for clarity
    significant_features = sorted_coefficients[
        (sorted_coefficients['Odds_Ratio'] > 1.1) | 
        (sorted_coefficients['Odds_Ratio'] < 0.9)
    ]
    
    # Create log scale for better visualization
    log_odds = np.log10(significant_features['Odds_Ratio'])
    colors = ['red' if o < 1 else 'blue' for o in significant_features['Odds_Ratio']]
    
    plt.barh(significant_features['Feature'], log_odds, color=colors)
    plt.axvline(x=0, color='black', linestyle='-', alpha=0.7)
    plt.xlabel('Log10(Odds Ratio)')
    plt.ylabel('Feature')
    plt.title('Logistic Regression Odds Ratios (Log10 Scale)')
    
    # Add odds ratio values as text
    for i, (_, row) in enumerate(significant_features.iterrows()):
        plt.text(
            np.log10(row['Odds_Ratio']) + (0.1 if row['Odds_Ratio'] < 1 else -0.1), 
            i, 
            f"{row['Odds_Ratio']:.2f}", 
            va='center', 
            ha='left' if row['Odds_Ratio'] < 1 else 'right',
            color='black', 
            fontweight='bold'
        )
    
    plt.grid(axis='x', alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_odds_ratios.png')
    
    return sorted_coefficients

def visualize_decision_boundary(log_reg, X_train, X_test, y_train, y_test, features):
    """
    Visualize the decision boundary for logistic regression using two most important features
    
    Parameters:
    log_reg: Trained logistic regression model
    X_train, X_test, y_train, y_test: Training and testing data
    features: Names of features
    """
    print("\nVisualizing decision boundary...")
    
    # Identify the two most important features
    feature_importance = np.abs(log_reg.coef_[0])
    top_indices = np.argsort(feature_importance)[-2:]
    top_features = [features[i] for i in top_indices]
    
    print(f"Creating decision boundary visualization using features: {top_features}")
    
    # Extract only these two features
    X_train_2d = X_train[top_features].values
    X_test_2d = X_test[top_features].values
    
    # Train a new model on just these two features
    log_reg_2d = LogisticRegression(max_iter=1000, random_state=42)
    log_reg_2d.fit(X_train_2d, y_train)
    
    # Create a meshgrid for visualization
    x_min, x_max = X_test_2d[:, 0].min() - 0.5, X_test_2d[:, 0].max() + 0.5
    y_min, y_max = X_test_2d[:, 1].min() - 0.5, X_test_2d[:, 1].max() + 0.5
    xx, yy = np.meshgrid(np.arange(x_min, x_max, 0.1),
                         np.arange(y_min, y_max, 0.1))
    
    # Predict class at each point in the meshgrid
    Z = log_reg_2d.predict(np.c_[xx.ravel(), yy.ravel()])
    Z = Z.reshape(xx.shape)
    
    # Also get probability predictions for coloring
    Z_proba = log_reg_2d.predict_proba(np.c_[xx.ravel(), yy.ravel()])[:, 1]
    Z_proba = Z_proba.reshape(xx.shape)
    
    # Create the plot
    plt.figure(figsize=(12, 10))
    
    # First subplot: Decision boundary with test points
    plt.subplot(2, 1, 1)
    plt.contourf(xx, yy, Z, alpha=0.4, cmap='RdBu')
    plt.scatter(X_test_2d[:, 0], X_test_2d[:, 1], c=y_test, 
               cmap='RdBu_r', edgecolors='k', alpha=0.8)
    plt.xlabel(top_features[0])
    plt.ylabel(top_features[1])
    plt.title('Logistic Regression Decision Boundary')
    plt.colorbar(label='Class')
    
    # Second subplot: Probability surface
    plt.subplot(2, 1, 2)
    contour = plt.contourf(xx, yy, Z_proba, alpha=0.8, cmap='RdBu_r')
    plt.colorbar(label='NLOS Probability')
    
    # Add the decision boundary
    plt.contour(xx, yy, Z_proba, [0.5], colors='k', linestyles='-', linewidths=2)
    
    # Plot test points
    plt.scatter(X_test_2d[:, 0], X_test_2d[:, 1], c=y_test, 
               cmap='RdBu_r', edgecolors='k', alpha=0.6)
    
    plt.xlabel(top_features[0])
    plt.ylabel(top_features[1])
    plt.title('Probability Surface with Decision Boundary')
    
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_boundary.png')

def interpret_results(log_reg, accuracy, auc_score, sorted_coefficients):
    """
    Interpret the results of the logistic regression model
    
    Parameters:
    log_reg: Trained logistic regression model
    accuracy: Model accuracy
    auc_score: AUC score
    sorted_coefficients: Sorted coefficients DataFrame
    """
    print("\nInterpreting results...")
    
    # Get top positive and negative features in terms of odds ratios
    top_positive = sorted_coefficients[sorted_coefficients['Odds_Ratio'] > 1].head(3)
    top_negative = sorted_coefficients[sorted_coefficients['Odds_Ratio'] < 1].head(3)
    
    print("\nLogistic Regression Interpretation:")
    print(f"- Logistic regression achieved an accuracy of {accuracy*100:.2f}%")
    print(f"- The AUC score is {auc_score:.4f}, indicating good discrimination ability")
    
    print("\n- Features most strongly associated with NLOS (odds ratio > 1):")
    for _, row in top_positive.iterrows():
        print(f"  * {row['Feature']}: Odds ratio = {row['Odds_Ratio']:.4f}")
        print(f"     (Being NLOS is {row['Odds_Ratio']:.2f}x more likely for each unit increase)")
    
    print("\n- Features most strongly associated with LOS (odds ratio < 1):")
    for _, row in top_negative.iterrows():
        print(f"  * {row['Feature']}: Odds ratio = {row['Odds_Ratio']:.4f}")
        print(f"     (Being NLOS is {row['Odds_Ratio']:.2f}x less likely for each unit increase)")
    
    # Calculate the odds at specific feature values
    print("\n- Interpreting odds for specific scenarios:")
    
    # Try to find SNR and Elevation in the features
    has_snr = 'SNR' in sorted_coefficients['Feature'].values
    has_elevation = 'Elevation' in sorted_coefficients['Feature'].values
    
    if has_snr and has_elevation:
        snr_coef = sorted_coefficients.loc[sorted_coefficients['Feature'] == 'SNR', 'Coefficient'].values[0]
        elevation_coef = sorted_coefficients.loc[sorted_coefficients['Feature'] == 'Elevation', 'Coefficient'].values[0]
        intercept = float(log_reg.intercept_)
        
        # Calculate log-odds for a few examples
        low_snr_low_el = intercept + snr_coef * 20 + elevation_coef * 10
        low_snr_high_el = intercept + snr_coef * 20 + elevation_coef * 60
        high_snr_low_el = intercept + snr_coef * 45 + elevation_coef * 10
        high_snr_high_el = intercept + snr_coef * 45 + elevation_coef * 60
        
        # Convert to probabilities
        prob_low_snr_low_el = 1 / (1 + np.exp(-low_snr_low_el))
        prob_low_snr_high_el = 1 / (1 + np.exp(-low_snr_high_el))
        prob_high_snr_low_el = 1 / (1 + np.exp(-high_snr_low_el))
        prob_high_snr_high_el = 1 / (1 + np.exp(-high_snr_high_el))
        
        print(f"\nPredicted NLOS probabilities for different scenarios:")
        print(f"- Low SNR (20 dB-Hz), Low Elevation (10°): {prob_low_snr_low_el:.2%}")
        print(f"- Low SNR (20 dB-Hz), High Elevation (60°): {prob_low_snr_high_el:.2%}")
        print(f"- High SNR (45 dB-Hz), Low Elevation (10°): {prob_high_snr_low_el:.2%}")
        print(f"- High SNR (45 dB-Hz), High Elevation (60°): {prob_high_snr_high_el:.2%}")
    
    # Create a summary figure
    plt.figure(figsize=(12, 8))
    
    # Set up 2x2 subplots
    plt.subplot(2, 2, 1)
    plt.bar(['Accuracy'], [accuracy])
    plt.ylim(0, 1)
    plt.title('Model Accuracy')
    plt.grid(axis='y', alpha=0.3)
    
    plt.subplot(2, 2, 2)
    plt.bar(['AUC Score'], [auc_score])
    plt.ylim(0, 1)
    plt.title('AUC Score')
    plt.grid(axis='y', alpha=0.3)
    
    plt.subplot(2, 2, 3)
    if not top_positive.empty:
        plt.barh(top_positive['Feature'], np.log(top_positive['Odds_Ratio']), color='red')
        plt.title('Top Features Increasing NLOS Odds (log scale)')
    else:
        plt.text(0.5, 0.5, 'No features with odds ratio > 1', ha='center', va='center')
    plt.grid(axis='x', alpha=0.3)
    
    plt.subplot(2, 2, 4)
    if not top_negative.empty:
        plt.barh(top_negative['Feature'], np.log(1/top_negative['Odds_Ratio']), color='blue')
        plt.title('Top Features Decreasing NLOS Odds (log scale)')
    else:
        plt.text(0.5, 0.5, 'No features with odds ratio < 1', ha='center', va='center')
    plt.grid(axis='x', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('figures/logistic_regression_summary.png')

def main():
    """
    Main function to run the logistic regression implementation
    """
    print("Exercise 3: Implementing Logistic Regression for NLOS Detection\n")
    
    # Load and preprocess data
    X_train, X_test, y_train, y_test, feature_names, original_X = load_and_preprocess_data()
    
    # Implement logistic regression
    log_reg, y_pred, y_pred_proba = implement_logistic_regression(
        X_train, X_test, y_train, y_test
    )
    
    # Evaluate the model
    accuracy, auc_score = evaluate_model(y_test, y_pred, y_pred_proba)
    
    # Analyze coefficients
    sorted_coefficients = analyze_coefficients(log_reg, feature_names)
    
    # Visualize decision boundary
    visualize_decision_boundary(log_reg, X_train, X_test, y_train, y_test, feature_names)
    
    # Interpret results - Now passing log_reg as the first parameter
    interpret_results(log_reg, accuracy, auc_score, sorted_coefficients)
    
    print("\nExercise 3 completed successfully!")
    print("Visualizations saved in the 'figures' directory.")

if __name__ == "__main__":
    main()