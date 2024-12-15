import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense, Dropout
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.optimizers import SGD, Adam
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # 或 'TkAgg'，视具体环境而定

# 设置随机种子
np.random.seed(1212)

# 加载数据
train_data = pd.read_csv('train.csv')
test_data = pd.read_csv('test.csv')


X = train_data.iloc[:, 1:].values / 255.0  # 归一化特征
y = pd.get_dummies(train_data.iloc[:, 0]).values  # 标签 One-Hot 编码

# 绘制前16张图片
fig, axes = plt.subplots(4, 4, figsize=(6, 6))
for i, ax in enumerate(axes.flat):
    if i < len(X):
        ax.imshow(X[i].reshape(28, 28), cmap='gray')  # 转换为28x28图像
        ax.set_title(f"Label: {np.argmax(y[i])}")  # 显示标签
        ax.axis('off')  # 隐藏坐标轴
plt.tight_layout()

# 保存图像
fig.savefig("mnist_samples.png")
print("图像已保存为 mnist_samples.png")

# 打印训练集和测试集的前五行
print("Train.csv:")
print(train_data.head())

print("\nTest.csv:")
print(test_data.head())

# 分离特征和标签
X = train_data.iloc[:, 1:].values / 255.0  # 特征归一化
y = to_categorical(train_data.iloc[:, 0].values, 10)  # 标签 One-Hot 编码

X_test = test_data.values / 255.0  # 测试数据归一化

# 分割训练集和验证集
X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=1212)

# 构建神经网络模型
def build_model(input_dim, hidden_layers, output_dim, dropout_rate=0.3):
    inp = Input(shape=(input_dim,))
    x = inp
    for i, units in enumerate(hidden_layers):
        x = Dense(units, activation='relu', name=f"Hidden_Layer_{i+1}")(x)
        if dropout_rate > 0:
            x = Dropout(dropout_rate)(x)
    out = Dense(output_dim, activation='softmax', name="Output_Layer")(x)
    return Model(inp, out)

# 模型训练函数
def train_model(model, X_train, y_train, X_val, y_val, optimizer, batch_size=100, epochs=20):
    model.compile(loss='categorical_crossentropy', optimizer=optimizer, metrics=['accuracy'])
    return model.fit(X_train, y_train, batch_size=batch_size, epochs=epochs, validation_data=(X_val, y_val), verbose=2)

# 构建不同优化器的模型
hidden_layers = [300, 100, 100, 200]
input_dim = 784
output_dim = 10

# SGD 优化器
model_sgd = build_model(input_dim, hidden_layers, output_dim)
history_sgd = train_model(model_sgd, X_train, y_train, X_val, y_val, SGD(learning_rate=0.1))

# Adam 优化器，学习率 0.1
model_adam = build_model(input_dim, hidden_layers, output_dim)
history_adam = train_model(model_adam, X_train, y_train, X_val, y_val, Adam(learning_rate=0.1))

# Adam 优化器，学习率 0.01
model_adam_lr01 = build_model(input_dim, hidden_layers, output_dim)
history_adam_lr01 = train_model(model_adam_lr01, X_train, y_train, X_val, y_val, Adam(learning_rate=0.01))

# Adam 优化器，学习率 0.5
model_adam_lr05 = build_model(input_dim, hidden_layers, output_dim)
history_adam_lr05 = train_model(model_adam_lr05, X_train, y_train, X_val, y_val, Adam(learning_rate=0.5))

# 添加 Dropout 的模型
model_dropout = build_model(input_dim, hidden_layers, output_dim, dropout_rate=0.3)
history_dropout = train_model(model_dropout, X_train, y_train, X_val, y_val, Adam(learning_rate=0.01))

# 生成预测结果并保存
preds = model_dropout.predict(X_test, batch_size=200)
preds = preds.argmax(axis=1)

submission = pd.DataFrame({'ImageId': range(1, len(preds) + 1), 'Label': preds})
submission.to_csv('mnist_submission.csv', index=False)

print(submission.head())

plt.figure(figsize=(12, 5))

# 绘制损失曲线
plt.subplot(1, 2, 1)
plt.plot(history_sgd.history['loss'], label='Train Loss (SGD)')
plt.plot(history_sgd.history['val_loss'], label='Validation Loss (SGD)')
plt.title('Loss During Training')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()

# 绘制准确率曲线
plt.subplot(1, 2, 2)
plt.plot(history_sgd.history['accuracy'], label='Train Accuracy (SGD)')
plt.plot(history_sgd.history['val_accuracy'], label='Validation Accuracy (SGD)')
plt.title('Accuracy During Training')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.legend()

plt.tight_layout()
plt.savefig("training_history_sgd.png")
