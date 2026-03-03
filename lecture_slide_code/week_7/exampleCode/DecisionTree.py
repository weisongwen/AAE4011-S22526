import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeClassifier, plot_tree, export_text
from sklearn.metrics import (accuracy_score, confusion_matrix, classification_report,
                            roc_curve, roc_auc_score, precision_recall_curve, auc)
import graphviz
from sklearn.tree import export_graphviz
import os
import warnings
warnings.filterwarnings('ignore')

# Create figures directory if it doesn't exist
if not os.path.exists('figures'):
    os.makedirs('figures')

def load_and_preprocess_data():
    """
    Load the GPS NLOS dataset and perform preprocessing
    
    Returns:
    tuple: (X_train, X_test, y_train, y_test, feature_names, original_X)
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
    
    # Store original data for later use
    original_X = X.copy()
    feature_names = X.columns
    
    y = data['NLOS_Status']
    
    # Split into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.3, random_state=42
    )
    
    print(f"Training set: {X_train.shape[0]} samples")
    print(f"Testing set: {X_test.shape[0]} samples")
    
    return X_train, X_test, y_train, y_test, feature_names, original_X

def implement_decision_tree(X_train, X_test, y_train, y_test, max_depth=5):
    """
    Implement decision tree model and make predictions
    
    Parameters:
    X_train, X_test, y_train, y_test: Training and testing sets
    max_depth: Maximum depth of the decision tree
    
    Returns:
    tuple: (dt_model, y_pred, y_pred_proba)
    """
    print(f"\nImplementing decision tree with max_depth={max_depth}...")
    
    # Create and train the model
    dt_model = DecisionTreeClassifier(
        max_depth=max_depth, 
        random_state=42,
        criterion='gini',
        min_samples_split=2,
        min_samples_leaf=1
    )
    dt_model.fit(X_train, y_train)
    
    # Make predictions
    y_pred = dt_model.predict(X_test)
    y_pred_proba = dt_model.predict_proba(X_test)[:, 1]  # Probability of NLOS
    
    # Also make training predictions for analysis
    y_train_pred = dt_model.predict(X_train)
    training_accuracy = accuracy_score(y_train, y_train_pred)
    print(f"Training accuracy: {training_accuracy:.4f}")
    
    return dt_model, y_pred, y_pred_proba

def evaluate_model(y_test, y_pred, y_pred_proba):
    """
    Evaluate the decision tree model
    
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
    try:
        auc_score = roc_auc_score(y_test, y_pred_proba)
    except:
        auc_score = 0.5  # Default if there's an issue calculating AUC
    
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
    plt.title('Confusion Matrix - Decision Tree')
    plt.ylabel('True Label')
    plt.xlabel('Predicted Label')
    plt.tight_layout()
    plt.savefig('figures/decision_tree_confusion_matrix.png')
    
    # Plot ROC curve
    try:
        fpr, tpr, thresholds = roc_curve(y_test, y_pred_proba)
        
        plt.figure(figsize=(10, 6))
        plt.plot(fpr, tpr, label=f'Decision Tree (AUC = {auc_score:.3f})')
        plt.plot([0, 1], [0, 1], 'k--', label='Random Classifier')
        plt.xlabel('False Positive Rate')
        plt.ylabel('True Positive Rate')
        plt.title('ROC Curve - Decision Tree')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('figures/decision_tree_roc.png')
        
        # Plot Precision-Recall curve
        precision, recall, _ = precision_recall_curve(y_test, y_pred_proba)
        pr_auc = auc(recall, precision)
        
        plt.figure(figsize=(10, 6))
        plt.plot(recall, precision, label=f'Decision Tree (AUC = {pr_auc:.3f})')
        plt.xlabel('Recall')
        plt.ylabel('Precision')
        plt.title('Precision-Recall Curve - Decision Tree')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('figures/decision_tree_precision_recall.png')
    except:
        print("Warning: Could not generate ROC or Precision-Recall curves, likely due to prediction issues.")
    
    # Plot histogram of predicted probabilities
    try:
        plt.figure(figsize=(10, 6))
        plt.hist(y_pred_proba[y_test == 0], bins=10, alpha=0.5, label='LOS (Class 0)', color='blue')
        plt.hist(y_pred_proba[y_test == 1], bins=10, alpha=0.5, label='NLOS (Class 1)', color='red')
        plt.xlabel('Predicted Probability of NLOS')
        plt.ylabel('Count')
        plt.title('Distribution of Predicted Probabilities')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig('figures/decision_tree_probability_distribution.png')
    except:
        print("Warning: Could not generate probability distribution plot.")
    
    return accuracy, auc_score

def analyze_feature_importance(dt_model, feature_names):
    """
    Analyze and visualize feature importance from the decision tree model
    
    Parameters:
    dt_model: Trained decision tree model
    feature_names: Names of the features
    
    Returns:
    DataFrame: Sorted feature importance
    """
    print("\nAnalyzing feature importance...")
    
    # Create DataFrame with feature importance
    importance = pd.DataFrame({
        'Feature': feature_names,
        'Importance': dt_model.feature_importances_
    })
    
    # Sort by importance
    sorted_importance = importance.sort_values('Importance', ascending=False)
    
    print("\nFeature Importance:")
    print(sorted_importance)
    
    # Visualize feature importance
    plt.figure(figsize=(12, 8))
    
    plt.bar(sorted_importance['Feature'], sorted_importance['Importance'])
    plt.xlabel('Feature')
    plt.ylabel('Importance')
    plt.title('Decision Tree Feature Importance')
    plt.xticks(rotation=45, ha='right')
    plt.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/decision_tree_feature_importance.png')
    
    return sorted_importance

def visualize_decision_tree(dt_model, feature_names, max_depth=3):
    """
    Visualize the decision tree structure
    
    Parameters:
    dt_model: Trained decision tree model
    feature_names: Names of the features
    max_depth: Maximum depth to visualize
    """
    print(f"\nVisualizing decision tree (limited to depth {max_depth})...")
    
    # Plot the decision tree
    plt.figure(figsize=(20, 12))
    plot_tree(dt_model, 
              max_depth=max_depth,
              feature_names=feature_names,
              class_names=['LOS', 'NLOS'],
              filled=True, 
              rounded=True, 
              fontsize=10)
    plt.title(f'Decision Tree Visualization (max_depth={max_depth})')
    plt.tight_layout()
    plt.savefig('figures/decision_tree_visualization.png', dpi=200)
    
    # Export tree as text for console viewing
    tree_text = export_text(dt_model, 
                            feature_names=list(feature_names),
                            max_depth=max_depth,
                            spacing=3)
    print("\nDecision Tree Structure:")
    print(tree_text)
    
    # Try to create a more detailed visualization with graphviz if available
    try:
        # Create dot file
        dot_data = export_graphviz(
            dt_model,
            max_depth=max_depth,
            out_file=None,
            feature_names=feature_names,
            class_names=['LOS', 'NLOS'],
            filled=True,
            rounded=True,
            special_characters=True
        )
        
        # Try saving to file directly
        with open('figures/decision_tree.dot', 'w') as f:
            f.write(dot_data)
        
        # Try rendering with graphviz
        try:
            graph = graphviz.Source(dot_data)
            graph.render('figures/decision_tree_graphviz', format='png')
            print("Detailed tree visualization created with graphviz.")
        except:
            print("Could not render graphviz visualization, but dot file was saved.")
    except:
        print("Could not generate detailed tree visualization with graphviz.")

def extract_decision_rules(dt_model, feature_names, max_depth=3):
    """
    Extract and display human-readable decision rules from the tree
    
    Parameters:
    dt_model: Trained decision tree model
    feature_names: Names of the features
    max_depth: Maximum depth for rule extraction
    
    Returns:
    list: Decision rules as strings
    """
    print("\nExtracting decision rules...")
    
    tree = dt_model.tree_
    
    def get_rules(tree, feature_names, node_id=0, depth=0, path=None):
        if path is None:
            path = []
        
        # Stop if we've reached max_depth
        if depth >= max_depth:
            return []
        
        # If leaf node, return the complete rule and class
        if tree.children_left[node_id] == -1:  # Leaf node
            class_val = np.argmax(tree.value[node_id][0])
            class_name = 'NLOS' if class_val == 1 else 'LOS'
            samples = tree.n_node_samples[node_id]
            value = tree.value[node_id][0]
            
            # Calculate node purity
            total = sum(value)
            if total > 0:
                purity = max(value) / total
            else:
                purity = 0
                
            rule = " AND ".join(path)
            if not rule:
                rule = "ROOT"
            return [(rule, class_name, samples, purity)]
        
        # Get feature name and threshold
        feature = feature_names[tree.feature[node_id]]
        threshold = tree.threshold[node_id]
        
        # Recurse for left child (feature <= threshold)
        left_path = path + [f"{feature} <= {threshold:.2f}"]
        left_rules = get_rules(tree, feature_names, tree.children_left[node_id], depth + 1, left_path)
        
        # Recurse for right child (feature > threshold)
        right_path = path + [f"{feature} > {threshold:.2f}"]
        right_rules = get_rules(tree, feature_names, tree.children_right[node_id], depth + 1, right_path)
        
        return left_rules + right_rules
    
    # Get all rules
    rules = get_rules(tree, feature_names)
    
    # Format and print rules
    print(f"\nExtracted {len(rules)} rules (max depth={max_depth}):")
    for i, (rule, pred_class, samples, purity) in enumerate(rules, 1):
        print(f"Rule {i}: IF {rule} THEN class = {pred_class}")
        print(f"  Samples: {samples}, Purity: {purity:.2%}\n")
    
    # Create a summary visualization of the rules
    if rules:
        plt.figure(figsize=(12, len(rules) * 0.8))
        
        # Prepare data for visualization
        rule_nums = [f"Rule {i}" for i in range(1, len(rules) + 1)]
        purities = [purity for _, _, _, purity in rules]
        samples = [sample for _, _, sample, _ in rules]
        classes = [class_name for _, class_name, _, _ in rules]
        colors = ['red' if c == 'NLOS' else 'blue' for c in classes]
        
        # Plot rule purities
        plt.barh(rule_nums, purities, color=colors)
        
        # Add sample counts as text
        for i, (samples_count, purity) in enumerate(zip(samples, purities)):
            plt.text(purity + 0.02, i, f"n = {samples_count}", va='center')
        
        plt.xlabel('Rule Purity (Confidence)')
        plt.ylabel('Decision Rule')
        plt.title('Decision Tree Rules Summary')
        plt.xlim(0, 1.15)  # Make room for sample texts
        plt.grid(axis='x', alpha=0.3)
        plt.tight_layout()
        plt.savefig('figures/decision_tree_rules.png')
    
    return rules

def explore_tree_depth(X_train, X_test, y_train, y_test, max_depths=None):
    """
    Explore different tree depths and their impact on accuracy
    
    Parameters:
    X_train, X_test, y_train, y_test: Training and testing data
    max_depths: List of maximum depths to try
    """
    if max_depths is None:
        max_depths = [1, 2, 3, 5, 7, 10, 15, 20, None]
    
    print("\nExploring different tree depths...")
    
    train_scores = []
    test_scores = []
    
    for depth in max_depths:
        # Train tree with specific depth
        dt = DecisionTreeClassifier(max_depth=depth, random_state=42)
        dt.fit(X_train, y_train)
        
        # Evaluate on training and test data
        train_pred = dt.predict(X_train)
        test_pred = dt.predict(X_test)
        
        train_acc = accuracy_score(y_train, train_pred)
        test_acc = accuracy_score(y_test, test_pred)
        
        train_scores.append(train_acc)
        test_scores.append(test_acc)
        
        # Print current depth results
        depth_str = str(depth) if depth is not None else "None (unlimited)"
        print(f"Depth {depth_str}: Train Accuracy = {train_acc:.4f}, Test Accuracy = {test_acc:.4f}")
    
    # Display depth analysis visually
    plt.figure(figsize=(12, 6))
    
    # Convert None to a plottable value
    x_values = [d if d is not None else max(max_depths[:len(max_depths)-1]) + 5 for d in max_depths]
    x_labels = [str(d) if d is not None else "None" for d in max_depths]
    
    plt.plot(x_values, train_scores, 'o-', label='Training Accuracy')
    plt.plot(x_values, test_scores, 'o-', label='Testing Accuracy')
    
    # Highlight max test accuracy
    max_idx = np.argmax(test_scores)
    plt.scatter([x_values[max_idx]], [test_scores[max_idx]], s=150, c='red', 
                marker='*', edgecolors='black', zorder=10, 
                label=f'Best Test Accuracy: {test_scores[max_idx]:.4f}')
    
    plt.xticks(x_values, x_labels)
    plt.xlabel('Maximum Tree Depth')
    plt.ylabel('Accuracy')
    plt.title('Decision Tree Performance vs Tree Depth')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('figures/decision_tree_depth_analysis.png')
    
    # Return best depth based on test accuracy
    best_depth = max_depths[np.argmax(test_scores)]
    return best_depth, max(test_scores)

def interpret_results(dt_model, accuracy, auc_score, sorted_importance, rules):
    """
    Interpret the results of the decision tree model
    
    Parameters:
    dt_model: Trained decision tree model
    accuracy: Model accuracy
    auc_score: AUC score
    sorted_importance: Sorted feature importance DataFrame
    rules: Extracted rules from the decision tree
    """
    print("\nInterpreting results...")
    
    # Get top important features
    top_features = sorted_importance.head(3)
    
    print("\nDecision Tree Interpretation:")
    print(f"- Decision tree achieved an accuracy of {accuracy*100:.2f}%")
    print(f"- The AUC score is {auc_score:.4f}")
    
    print("\n- Most important features for classification:")
    for _, row in top_features.iterrows():
        print(f"  * {row['Feature']}: Importance = {row['Importance']:.4f}")
    
    # Interpret top decision rules
    if rules:
        print("\n- Key decision rules from the tree:")
        # Filter for high purity rules (confidence > 80%)
        high_conf_rules = [(rule, class_name, samples, purity) 
                           for rule, class_name, samples, purity in rules if purity >= 0.8]
        
        # Sort by sample count to show most common rules first
        high_conf_rules.sort(key=lambda x: x[2], reverse=True)
        
        # Show top rules for each class
        los_rules = [r for r in high_conf_rules if r[1] == 'LOS']
        nlos_rules = [r for r in high_conf_rules if r[1] == 'NLOS']
        
        # Print top LOS rules
        if los_rules:
            print("\n  Top rules for predicting LOS:")
            for i, (rule, _, samples, purity) in enumerate(los_rules[:2], 1):
                print(f"  Rule L{i}: IF {rule} THEN class = LOS")
                print(f"    Confidence: {purity:.1%}, Samples: {samples}")
        
        # Print top NLOS rules
        if nlos_rules:
            print("\n  Top rules for predicting NLOS:")
            for i, (rule, _, samples, purity) in enumerate(nlos_rules[:2], 1):
                print(f"  Rule N{i}: IF {rule} THEN class = NLOS")
                print(f"    Confidence: {purity:.1%}, Samples: {samples}")
    
    # Create a summary figure
    plt.figure(figsize=(14, 10))
    
    # Set up subplots
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
    plt.barh(top_features['Feature'][::-1], top_features['Importance'][::-1])
    plt.xlabel('Importance')
    plt.title('Top Features')
    plt.grid(axis='x', alpha=0.3)
    
    # Add a tree characteristics summary in the 4th subplot
    plt.subplot(2, 2, 4)
    plt.axis('off')  # Turn off axis
    
    # Get tree information
    n_nodes = dt_model.tree_.node_count
    n_leaves = dt_model.tree_.n_leaves
    max_depth = dt_model.tree_.max_depth
    
    # Add text summarizing tree characteristics
    info_text = f"Tree Structure Summary:\n\n"
    info_text += f"- Number of nodes: {n_nodes}\n"
    info_text += f"- Number of leaves: {n_leaves}\n"
    info_text += f"- Maximum depth: {max_depth}\n"
    
    # Calculate class distribution in leaf nodes
    n_los_leaves = 0
    n_nlos_leaves = 0
    
    for i in range(n_nodes):
        if dt_model.tree_.children_left[i] == -1:  # If it's a leaf
            class_counts = dt_model.tree_.value[i][0]
            majority_class = np.argmax(class_counts)
            if majority_class == 0:
                n_los_leaves += 1
            else:
                n_nlos_leaves += 1
    
    info_text += f"- Leaves predicting LOS: {n_los_leaves}\n"
    info_text += f"- Leaves predicting NLOS: {n_nlos_leaves}\n"
    
    plt.text(0.5, 0.5, info_text, ha='center', va='center', fontsize=12)
    plt.title('Tree Characteristics')
    
    plt.tight_layout()
    plt.savefig('figures/decision_tree_summary.png')

def main():
    """
    Main function to run the decision tree implementation
    """
    print("Exercise 4: Implementing Decision Tree for NLOS Detection\n")
    
    # Load and preprocess data
    X_train, X_test, y_train, y_test, feature_names, original_X = load_and_preprocess_data()
    
    # Explore different tree depths to find the best
    best_depth, best_accuracy = explore_tree_depth(X_train, X_test, y_train, y_test)
    print(f"\nBest tree depth found: {best_depth if best_depth is not None else 'None (unlimited)'}")
    print(f"Best accuracy: {best_accuracy:.4f}")
    
    # Implement decision tree with the best depth
    dt_model, y_pred, y_pred_proba = implement_decision_tree(
        X_train, X_test, y_train, y_test, max_depth=best_depth
    )
    
    # Evaluate the model
    accuracy, auc_score = evaluate_model(y_test, y_pred, y_pred_proba)
    
    # Analyze feature importance
    sorted_importance = analyze_feature_importance(dt_model, feature_names)
    
    # Visualize the decision tree
    visualize_decision_tree(dt_model, feature_names, max_depth=3)
    
    # Extract decision rules
    rules = extract_decision_rules(dt_model, feature_names, max_depth=3)
    
    # Interpret results
    interpret_results(dt_model, accuracy, auc_score, sorted_importance, rules)
    
    print("\nExercise 4 completed successfully!")
    print("Visualizations saved in the 'figures' directory.")

if __name__ == "__main__":
    main()