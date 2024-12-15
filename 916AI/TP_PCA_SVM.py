import numpy as np
import matplotlib
matplotlib.use('TkAgg') 
import matplotlib.pyplot as plt
import pandas as pd
from sklearn.svm import SVC

#%% Block 1
data = pd.read_csv("flowerTrain.data", names=['sepallength', 'sepalwidth', 'petallength', 'petalwidth', 'species'])
x = data.iloc[:, 0:4]
plt.scatter(data.iloc[:, 0], data.iloc[:, 1], alpha=0.2, s=100 * data.iloc[:, 2], c=100 * data.iloc[:, 3])
plt.show()


#%% Block 2.1
X1 = np.array([[1,5,3,3],
               [4,4,3,5]])
X2 = np.array([[1.268,4.732,3.500,2.500],
               [3.000,5.000,3.134,4.866]])

def PCA(X, reducedDim, axis=0):
    if isinstance(X, pd.DataFrame):
        X = X.to_numpy()

    # 1. Compute the mean
    mean = np.mean(X, axis=axis, keepdims=True)
    X_centered = X - mean  # Center the data

    # 2. Compute the covariance matrix (manual formula)
    if axis == 0:  # Compute by columns
        covariance_matrix = np.dot(X_centered.T, X_centered) / (X_centered.shape[0] - 1)
    else:  # Compute by rows
        covariance_matrix = np.dot(X_centered, X_centered.T) / (X_centered.shape[1] - 1)

    # 3. Perform eigen decomposition
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)

    # 4. Sort by descending eigenvalues
    sorted_indices = np.argsort(eigenvalues)[::-1]
    eigenvectors = eigenvectors[:, sorted_indices[:reducedDim]]
    eigenvalues = eigenvalues[sorted_indices[:reducedDim]]

    # 5. Project data onto principal components
    if axis == 0:
        reduced_X = np.dot(X_centered, eigenvectors)
    else:
        reduced_X = np.dot(eigenvectors.T, X_centered)

    return reduced_X, eigenvectors, eigenvalues



# Call the PCA function and reduce to 1 dimension
reduced_X, eigenvectors, eigenvalues = PCA(X2, reducedDim=1,axis =1)

# Plot the original data points
plt.scatter(X2[0, :], X2[1, :], color='blue', label='Original Data')

# Plot the principal component directions
origin = np.mean(X2, axis=1)  # Central point
for i in range(eigenvectors.shape[1]):
    plt.arrow(origin[0], origin[1],
              eigenvectors[0, i] * eigenvalues[i],
              eigenvectors[1, i] * eigenvalues[i],
              color='red', head_width=0.2, label=f'Eigenvector {i+1}' if i == 0 else None)

plt.axhline(0, color='gray', linestyle='--')
plt.axvline(0, color='gray', linestyle='--')
plt.title('PCA on 2D Data')
plt.xlabel('Feature 1')
plt.ylabel('Feature 2')
plt.legend()
plt.grid()
plt.show()


#%% Block 2.2
data = pd.read_csv("flowerTrain.data", names=['sepallength', 'sepalwidth', 'petallength', 'petalwidth', 'species'])
x = data.iloc[:, 0:4]
x_numpy = x.to_numpy()  # Ensure the data is in NumPy array format
reduced_X, eigenvectors, eigenvalues = PCA(x_numpy, reducedDim=2, axis=0)

# Debugging information
print("Original data shape:", x_numpy.shape)  # Should be (144, 4)
print("Reduced data shape:", reduced_X.shape)  # Should be (144, 2)


# Add the reduced dimensions to the DataFrame
data['PC1'] = reduced_X[:, 0]
data['PC2'] = reduced_X[:, 1]

# Plot the reduced data
plt.figure(figsize=(10, 6))
for species in data['species'].unique():
    subset = data[data['species'] == species]
    plt.scatter(subset['PC1'], subset['PC2'], label=species, alpha=0.7)
plt.title("PCA: Flower Dataset Reduced to 2D")
plt.xlabel("Principal Component 1")
plt.ylabel("Principal Component 2")
plt.legend()
plt.grid()
plt.show()


#%% Block 3
# Binary label conversion
data['binary_label'] = data['species'].apply(lambda x: 1 if x == 'interior' else -1)

# Extract the reduced features and labels
X = data[['PC1', 'PC2']].to_numpy()  # Feature matrix
y = data['binary_label'].to_numpy()  # Labels
def svm_model(X, y, C=1E10, plot_decision_boundary=False):

    from sklearn.svm import SVC
    import matplotlib.pyplot as plt
    import numpy as np
    
    # Train the SVM model
    svm = SVC(kernel='linear', C=C)
    svm.fit(X, y)  # Model training
    
    if plot_decision_boundary:
        # Decision boundary formula
        w = svm.coef_[0]  # Weight vector [w1, w2]
        b = svm.intercept_[0]  # Bias term

        # Decision boundary formula
        x_vals = np.linspace(X[:, 0].min() - 1, X[:, 0].max() + 1, 100)
        y_decision = -(w[0] / w[1]) * x_vals - b / w[1]  # Decision boundary
        y_support_up = -(w[0] / w[1]) * x_vals - (b - 1) / w[1]  # Upper boundary
        y_support_down = -(w[0] / w[1]) * x_vals - (b + 1) / w[1]  # Lower boundary

        # Define the grid range (for background shading)
        x_min, x_max = X[:, 0].min() - 1, X[:, 0].max() + 1
        y_min, y_max = X[:, 1].min() - 1, X[:, 1].max() + 1
        xx, yy = np.meshgrid(np.linspace(x_min, x_max, 1000),
                            np.linspace(y_min, y_max, 1000))
        grid = np.c_[xx.ravel(), yy.ravel()]
        Z = svm.decision_function(grid).reshape(xx.shape)

        # Plot the decision boundary, support vector margins, and three species of data points
        plt.figure(figsize=(10, 6))
        plt.xlim(-4, 4)
        plt.ylim(-2, 2)


        # Plot the decision boundary
        plt.plot(x_vals, y_decision, 'r-', linewidth=2, label="Decision Boundary")
        plt.plot(x_vals, y_support_up, 'r--', linewidth=1, label="Support Vector Boundary (+1)")
        plt.plot(x_vals, y_support_down, 'r--', linewidth=1, label="Support Vector Boundary (-1)")


        # Plot three species of data points
        for species in data['species'].unique():
            subset = data[data['species'] == species]
            plt.scatter(subset['PC1'], subset['PC2'], label=species, alpha=0.7)

        # Plot support vectors
        plt.scatter(svm.support_vectors_[:, 0], svm.support_vectors_[:, 1],
                    s=100, facecolors='none', edgecolors='k', label="Support Vectors")

        # Add legend and labels
        plt.title("SVM Decision Boundary with Support Vector Margins")
        plt.xlabel("Principal Component 1")
        plt.ylabel("Principal Component 2")
        plt.legend()  # Display legend
        plt.grid()  # Display grid
        

    return svm
svm = svm_model(X, y, C=1E10, plot_decision_boundary=True)
plt.show()
#%% Block 4


# Define new sample data
new_samples = np.array([
    [5.1, 3.5, 1.4, 0.2],
    [7.0, 3.2, 4.7, 1.4],
    [6.4, 3.2, 4.5, 1.5],
    [6.3, 3.3, 6.0, 2.5],
    [5.8, 2.7, 5.1, 1.9],
    [4.9, 3.0, 1.4, 0.2]
])


reduced_new, eigenvectors, eigenvalues = PCA(new_samples, reducedDim=2, axis=0)

print("Original data shape:", new_samples.shape)  
print("Reduced data shape:", reduced_new.shape) 


# Plot new sample points onto the existing graph
new_samples_df = pd.DataFrame(reduced_new, columns=['PC1', 'PC2'])
svm = svm_model(X, y, C=1E10, plot_decision_boundary=True)
# Ensure the plot remains open
plt.scatter(new_samples_df['PC1'], new_samples_df['PC2'], c='orange', edgecolors='k', s=200, label="New Samples", marker='*')

# Update the legend
plt.legend()  # Add legend for new samples



plt.show()  # Display the graph
