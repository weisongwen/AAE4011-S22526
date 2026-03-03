# Exercise 6: Model Comparison and Real-World Application

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import accuracy_score, roc_auc_score, classification_report, confusion_matrix
from sklearn.model_selection import train_test_split
import os

# Create figures directory if it doesn't exist
if not os.path.exists('figures'):
    os.makedirs('figures')

def compare_models(models_dict, X_test, y_test):
    """
    Compare the performance of multiple models
    
    Parameters:
    models_dict: Dictionary of {model_name: model_object}
    X_test: Test features
    y_test: Test labels
    
    Returns:
    DataFrame: Performance comparison
    """
    print("\nComparing model performance...")
    
    # Initialize lists to store results
    model_names = []
    accuracy_scores = []
    auc_scores = []
    
    # For each model, compute metrics
    for name, model in models_dict.items():
        model_names.append(name)
        
        # Handle different model types
        if name == 'Linear Regression':
            y_pred_proba = model.predict(X_test)
            y_pred = (y_pred_proba > 0.5).astype(int)
        else:
            y_pred = model.predict(X_test)
            y_pred_proba = model.predict_proba(X_test)[:, 1]
        
        # Calculate metrics
        accuracy = accuracy_score(y_test, y_pred)
        auc = roc_auc_score(y_test, y_pred_proba)
        
        accuracy_scores.append(accuracy)
        auc_scores.append(auc)
        
        # Print individual model performance
        print(f"\n{name} Performance:")
        print(f"  Accuracy: {accuracy:.4f}")
        print(f"  AUC: {auc:.4f}")
        print(f"  Classification Report:")
        print(classification_report(y_test, y_pred))
    
    # Create comparison DataFrame
    comparison_df = pd.DataFrame({
        'Model': model_names,
        'Accuracy': accuracy_scores,
        'AUC': auc_scores
    })
    
    # Sort by AUC (or could use accuracy)
    comparison_df = comparison_df.sort_values('AUC', ascending=False)
    
    print("\nModel Performance Comparison:")
    print(comparison_df)
    
    # Create bar chart comparison
    plt.figure(figsize=(12, 6))
    
    x = np.arange(len(model_names))
    width = 0.35
    
    plt.bar(x - width/2, accuracy_scores, width, label='Accuracy')
    plt.bar(x + width/2, auc_scores, width, label='AUC')
    
    plt.xlabel('Model')
    plt.ylabel('Score')
    plt.title('Model Performance Comparison')
    plt.xticks(x, model_names, rotation=45, ha='right')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/model_comparison.png')
    
    # Create ROC curve comparison
    plt.figure(figsize=(10, 8))
    
    for name, model in models_dict.items():
        if name == 'Linear Regression':
            y_pred_proba = model.predict(X_test)
        else:
            y_pred_proba = model.predict_proba(X_test)[:, 1]
        
        from sklearn.metrics import roc_curve
        fpr, tpr, _ = roc_curve(y_test, y_pred_proba)
        auc = roc_auc_score(y_test, y_pred_proba)
        
        plt.plot(fpr, tpr, label=f'{name} (AUC = {auc:.3f})')
    
    plt.plot([0, 1], [0, 1], 'k--', label='Random Classifier')
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    plt.title('ROC Curves Comparison')
    plt.legend(loc='lower right')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/model_comparison_roc.png')
    
    return comparison_df

def generate_urban_path_data(n_points=200):
    """
    Generate synthetic GPS data for a path through an urban area
    
    Parameters:
    n_points: Number of data points to generate
    
    Returns:
    DataFrame: Synthetic path data
    """
    print("\nGenerating synthetic urban path data...")
    
    # Set random seed for reproducibility
    np.random.seed(42)
    
    # Time in seconds
    time = np.arange(n_points)
    
    # Generate synthetic features
    snr = np.random.normal(35, 10, n_points)  # Average SNR with noise
    constellation = np.random.choice([1, 2, 3, 4], n_points)  # Random constellation
    
    # Create a pattern for elevation (satellites movement)
    elevation = 45 + 15 * np.sin(time / 30) + np.random.normal(0, 5, n_points)
    elevation = np.clip(elevation, 5, 90)
    
    # Create azimuth angles that change as the user moves
    azimuth = (time * 3) % 360 + np.random.normal(0, 5, n_points)
    
    # Create buildings at certain azimuths to cause NLOS
    building_azimuths = [30, 150, 270]
    building_widths = [20, 30, 15]
    
    # Start with all LOS
    nlos_status = np.zeros(n_points)
    
    # Add NLOS based on buildings and low elevation
    for i in range(n_points):
        # Check if satellite is behind a building
        for b_azimuth, b_width in zip(building_azimuths, building_widths):
            # If azimuth is pointing toward a building and elevation is not too high
            az_diff = min(abs(azimuth[i] - b_azimuth), 360 - abs(azimuth[i] - b_azimuth))
            if az_diff < b_width/2 and elevation[i] < 30:
                nlos_status[i] = 1
        
        # Additional factors that make NLOS more likely
        if elevation[i] < 15:  # Low elevation often means NLOS
            if np.random.random() < 0.7:  # 70% chance of NLOS if very low elevation
                nlos_status[i] = 1
        elif snr[i] < 25:  # Low SNR is often NLOS
            if np.random.random() < 0.6:  # 60% chance of NLOS if low SNR
                nlos_status[i] = 1
    
    # Create dataframe
    path_df = pd.DataFrame({
        'Time': time,
        'SNR': snr,
        'Constellation': constellation,
        'Elevation': elevation,
        'Azimuth': azimuth,
        'NLOS_Status': nlos_status
    })
    
    print(f"Generated {n_points} points of path data")
    print(f"NLOS percentage: {path_df['NLOS_Status'].mean() * 100:.1f}%")
    
    return path_df

def prepare_path_data(path_data, feature_columns):
    """
    Prepare path data for model prediction
    
    Parameters:
    path_data: Path DataFrame
    feature_columns: Feature column names used in training
    
    Returns:
    DataFrame: Processed features ready for prediction
    """
    # Extract features
    path_features = path_data[['SNR', 'Constellation', 'Elevation', 'Azimuth']]
    
    # One-hot encode Constellation
    path_features = pd.get_dummies(
        path_features, columns=['Constellation'], drop_first=True
    )
    
    # Add any missing columns that the model was trained on
    for col in feature_columns:
        if col not in path_features.columns:
            path_features[col] = 0
    
    # Ensure columns are in the same order as during training
    path_features = path_features[feature_columns]
    
    return path_features

def apply_model_to_path(best_model, path_data, feature_columns):
    """
    Apply the best model to the synthetic path data
    
    Parameters:
    best_model: Best performing model
    path_data: Path DataFrame
    feature_columns: Feature column names used in training
    
    Returns:
    DataFrame: Path data with predictions
    """
    print("\nApplying best model to path data...")
    
    # Prepare features
    path_features = prepare_path_data(path_data, feature_columns)
    
    # Make predictions
    path_data['NLOS_Predicted'] = best_model.predict(path_features)
    path_data['NLOS_Probability'] = best_model.predict_proba(path_features)[:, 1]
    
    # Calculate accuracy
    accuracy = accuracy_score(path_data['NLOS_Status'], path_data['NLOS_Predicted'])
    print(f"Accuracy on synthetic path: {accuracy:.4f}")
    
    # Create visualizations
    
    # 1. Path visualization over time
    plt.figure(figsize=(15, 10))
    
    # Plot 1: Path with actual NLOS status
    plt.subplot(2, 1, 1)
    plt.scatter(path_data['Time'], path_data['Elevation'], 
               c=path_data['NLOS_Status'], cmap='coolwarm', 
               alpha=0.8, s=50)
    plt.colorbar(label='Actual NLOS Status')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Elevation Angle (degrees)')
    plt.title('Actual NLOS Status Along Path')
    plt.grid(True, alpha=0.3)
    
    # Plot 2: Path with predicted NLOS probability
    plt.subplot(2, 1, 2)
    scatter = plt.scatter(path_data['Time'], path_data['Elevation'], 
                         c=path_data['NLOS_Probability'], cmap='coolwarm', 
                         alpha=0.8, s=50, vmin=0, vmax=1)
    plt.colorbar(scatter, label='Predicted NLOS Probability')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Elevation Angle (degrees)')
    plt.title('Predicted NLOS Probability Along Path')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('figures/path_nlos_prediction.png')
    
    # 2. Sky plot visualization (polar plot)
    plt.figure(figsize=(10, 10))
    ax = plt.subplot(111, projection='polar')
    
    # Convert azimuth from degrees to radians
    azimuth_rad = np.deg2rad(path_data['Azimuth'])
    
    # Create scatter plot
    sc = ax.scatter(azimuth_rad, 90 - path_data['Elevation'],  # Convert elevation to radial distance from center
                   c=path_data['NLOS_Probability'], cmap='coolwarm',
                   alpha=0.8, s=50, vmin=0, vmax=1)
    
    # Set plot properties
    ax.set_theta_zero_location('N')  # Set 0 degrees to North
    ax.set_theta_direction(-1)  # Clockwise
    ax.set_rlabel_position(0)  # Move radial labels away from plot
    
    # Set custom radial limits and labels
    ax.set_rmax(90)
    ax.set_rticks([0, 30, 60, 90])
    ax.set_yticklabels(['90°', '60°', '30°', '0°'])  # Elevation labels (90° at center)
    
    plt.colorbar(sc, label='NLOS Probability')
    plt.title('NLOS Probability by Azimuth and Elevation', y=1.08)
    plt.tight_layout()
    plt.savefig('figures/nlos_skyplot.png')
    
    return path_data

def simulate_positioning_improvement(path_data):
    """
    Simulate positioning improvement with NLOS detection
    
    Parameters:
    path_data: Path DataFrame with NLOS predictions
    
    Returns:
    DataFrame: Path data with simulated positioning errors
    """
    print("\nSimulating positioning improvement with NLOS detection...")
    
    # Set random seed for reproducibility
    np.random.seed(42)
    
    # Create synthetic positioning errors
    # LOS signals: small errors, NLOS signals: large errors
    path_data['Position_Error_Without_NLOS_Detection'] = np.where(
        path_data['NLOS_Status'] == 1,
        np.random.normal(15, 5, len(path_data)),  # Large errors for NLOS signals
        np.random.normal(2, 1, len(path_data))    # Small errors for LOS signals
    )
    
    # With NLOS detection: exclude high probability NLOS signals
    path_data['Used_For_Position'] = path_data['NLOS_Probability'] < 0.6
    
    # Calculate position error with NLOS detection
    # If we have enough satellites after filtering (at least 4)
    min_satellites = 4  # Minimum satellites needed for 3D positioning
    total_satellites = 10  # Assume 10 satellites visible at each time point
    path_data['Available_Satellites'] = total_satellites
    
    position_errors = []
    satellites_used = []
    
    # For each time point, simulate positioning error based on satellites used
    for i in range(len(path_data)):
        # Use a rolling window to count satellites used after NLOS filtering
        # This simulates having multiple satellites in view at each time point
        window_start = max(0, i - 9)  # Look at current point and previous 9 points (total 10)
        usable_sats = path_data['Used_For_Position'][window_start:i+1].sum()
        
        # Normalize by window size
        window_size = min(10, i + 1)
        usable_sats = int(usable_sats * (total_satellites / window_size))
        satellites_used.append(usable_sats)
        
        if usable_sats >= min_satellites:
            # Good position fix with only LOS signals
            error = np.random.normal(2, 1)
        else:
            # Have to use some NLOS signals
            # Error proportional to how many NLOS signals we need to use
            shortage = min_satellites - usable_sats
            error_scale = 2 + (shortage * 3)  # Error increases with more NLOS signals needed
            error = np.random.normal(error_scale, error_scale/2)
        
        position_errors.append(error)
    
    path_data['Satellites_Used'] = satellites_used
    path_data['Position_Error_With_NLOS_Detection'] = position_errors
    
    # Calculate statistics
    avg_error_without = path_data['Position_Error_Without_NLOS_Detection'].mean()
    avg_error_with = path_data['Position_Error_With_NLOS_Detection'].mean()
    improvement = (1 - avg_error_with / avg_error_without) * 100
    
    print(f"Average position error without NLOS detection: {avg_error_without:.2f} meters")
    print(f"Average position error with NLOS detection: {avg_error_with:.2f} meters")
    print(f"Positioning accuracy improvement: {improvement:.1f}%")
    
    # Visualize positioning improvement
    plt.figure(figsize=(12, 8))
    
    # Upper plot: Position errors
    plt.subplot(2, 1, 1)
    plt.plot(path_data['Time'], path_data['Position_Error_Without_NLOS_Detection'], 
             'r-', label='Without NLOS Detection')
    plt.plot(path_data['Time'], path_data['Position_Error_With_NLOS_Detection'], 
             'g-', label='With NLOS Detection')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Position Error (meters)')
    plt.title('Impact of NLOS Detection on Positioning Accuracy')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Lower plot: Satellites used vs available
    plt.subplot(2, 1, 2)
    plt.plot(path_data['Time'], path_data['Satellites_Used'], 'b-', label='Satellites Used After NLOS Filtering')
    plt.axhline(y=min_satellites, color='r', linestyle='--', label=f'Minimum Required ({min_satellites})')
    plt.axhline(y=total_satellites, color='g', linestyle='--', label=f'Total Available ({total_satellites})')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Number of Satellites')
    plt.title('Satellites Used After NLOS Filtering')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('figures/positioning_improvement.png')
    
    # Create a histogram of positioning errors
    plt.figure(figsize=(10, 6))
    plt.hist(path_data['Position_Error_Without_NLOS_Detection'], bins=20, alpha=0.5, label='Without NLOS Detection')
    plt.hist(path_data['Position_Error_With_NLOS_Detection'], bins=20, alpha=0.5, label='With NLOS Detection')
    plt.xlabel('Position Error (meters)')
    plt.ylabel('Frequency')
    plt.title('Distribution of Position Errors')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig('figures/position_error_distribution.png')
    
    return path_data

def create_application_summary(path_data):
    """
    Create a summary of the application results
    
    Parameters:
    path_data: Path DataFrame with simulation results
    """
    print("\nCreating application summary...")
    
    # Calculate key statistics
    avg_error_without = path_data['Position_Error_Without_NLOS_Detection'].mean()
    avg_error_with = path_data['Position_Error_With_NLOS_Detection'].mean()
    
    max_error_without = path_data['Position_Error_Without_NLOS_Detection'].max()
    max_error_with = path_data['Position_Error_With_NLOS_Detection'].max()
    
    p95_error_without = np.percentile(path_data['Position_Error_Without_NLOS_Detection'], 95)
    p95_error_with = np.percentile(path_data['Position_Error_With_NLOS_Detection'], 95)
    
    avg_satellites = path_data['Satellites_Used'].mean()
    min_satellites = path_data['Satellites_Used'].min()
    
    # Create a figure with multiple subplots summarizing the results
    plt.figure(figsize=(12, 10))
    
    # Subplot 1: Average and 95th percentile errors
    plt.subplot(2, 2, 1)
    metrics = ['Average Error', '95th Percentile Error', 'Maximum Error']
    without_values = [avg_error_without, p95_error_without, max_error_without]
    with_values = [avg_error_with, p95_error_with, max_error_with]
    
    x = np.arange(len(metrics))
    width = 0.35
    
    plt.bar(x - width/2, without_values, width, label='Without NLOS Detection')
    plt.bar(x + width/2, with_values, width, label='With NLOS Detection')
    plt.ylabel('Error (meters)')
    plt.title('Positioning Error Metrics')
    plt.xticks(x, metrics)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Subplot 2: Satellite usage
    plt.subplot(2, 2, 2)
    labels = ['Used after filtering', 'Filtered out (NLOS)']
    sizes = [avg_satellites, 10 - avg_satellites]  # Assuming 10 total satellites
    
    plt.pie(sizes, labels=labels, autopct='%1.1f%%', startangle=90, colors=['lightgreen', 'lightcoral'])
    plt.axis('equal')
    plt.title('Average Satellite Utilization')
    
    # Subplot 3: NLOS Detection Performance
    plt.subplot(2, 2, 3)
    
    tn, fp, fn, tp = confusion_matrix(path_data['NLOS_Status'], path_data['NLOS_Predicted']).ravel()
    
    conf_data = [
        ['True Negatives', tn],
        ['False Positives', fp],
        ['False Negatives', fn],
        ['True Positives', tp]
    ]
    
    table = plt.table(cellText=conf_data, loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1, 2)
    plt.axis('off')
    plt.title('NLOS Detection Confusion Matrix')
    
    # Subplot 4: Improvement summary
    plt.subplot(2, 2, 4)
    plt.axis('off')
    
    improvement = (1 - avg_error_with / avg_error_without) * 100
    
    summary_text = (
        f"NLOS Detection and Positioning Results\n\n"
        f"• Detection accuracy: {accuracy_score(path_data['NLOS_Status'], path_data['NLOS_Predicted']):.1%}\n"
        f"• Average satellites used: {avg_satellites:.1f} of 10 available\n"
        f"• Positioning improvement: {improvement:.1f}%\n"
        f"• Average error reduction: {avg_error_without - avg_error_with:.2f} meters\n"
        f"• 95th percentile error reduction: {p95_error_without - p95_error_with:.2f} meters"
    )
    
    plt.text(0.5, 0.5, summary_text, ha='center', va='center', fontsize=12)
    
    plt.tight_layout()
    plt.savefig('figures/application_summary.png')
    
    print("Application summary complete.")
    print(f"Positioning improvement: {improvement:.1f}%")
    print(f"Average error reduction: {avg_error_without - avg_error_with:.2f} meters")

def main():
    """
    Main function for Exercise 6: Model Comparison and Real-World Application
    """
    print("Exercise 6: Model Comparison and Real-World Application")
    
    # Step 1: Load the previous models
    # Assume we've implemented these in previous exercises
    from sklearn.linear_model import LinearRegression, LogisticRegression
    from sklearn.tree import DecisionTreeClassifier
    from sklearn.svm import SVC
    
    # Load data
    data = pd.read_csv('gps_nlos_dataset.csv')
    
    # Prepare data
    X = data[['SNR', 'Constellation', 'Elevation', 'Azimuth']]
    y = data['NLOS_Status']
    
    # Convert Constellation to one-hot encoding
    X = pd.get_dummies(X, columns=['Constellation'], drop_first=True)
    
    # Split data
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
    
    # Train models
    # Linear Regression
    lin_reg = LinearRegression()
    lin_reg.fit(X_train, y_train)
    
    # Logistic Regression
    log_reg = LogisticRegression(max_iter=1000, random_state=42)
    log_reg.fit(X_train, y_train)
    
    # Decision Tree
    dt_clf = DecisionTreeClassifier(max_depth=5, random_state=42)
    dt_clf.fit(X_train, y_train)
    
    # SVM (linear)
    svm_linear = SVC(kernel='linear', probability=True, random_state=42)
    svm_linear.fit(X_train, y_train)
    
    # SVM (RBF)
    svm_rbf = SVC(kernel='rbf', probability=True, random_state=42)
    svm_rbf.fit(X_train, y_train)
    
    # Store models in dictionary
    models = {
        'Linear Regression': lin_reg,
        'Logistic Regression': log_reg,
        'Decision Tree': dt_clf,
        'SVM (Linear)': svm_linear,
        'SVM (RBF)': svm_rbf
    }
    
    # Step 2: Compare models
    comparison_df = compare_models(models, X_test, y_test)
    
    # Step 3: Identify best model
    best_model_name = comparison_df.iloc[0]['Model']
    best_model = models[best_model_name]
    print(f"\nBest model: {best_model_name}")
    
    # Step 4: Generate synthetic path data
    path_data = generate_urban_path_data(200)
    
    # Step 5: Apply best model to path data
    path_data = apply_model_to_path(best_model, path_data, X.columns)
    
    # Step 6: Simulate positioning improvement
    path_data = simulate_positioning_improvement(path_data)
    
    # Step 7: Create application summary
    create_application_summary(path_data)
    
    print("\nExercise 6 completed successfully!")
    print("All visualizations saved in the 'figures' directory.")

if __name__ == "__main__":
    main()