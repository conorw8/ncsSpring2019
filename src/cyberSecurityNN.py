from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.layers import Embedding
from keras.layers import LSTM
from keras.optimizers import Adam
from keras.models import model_from_yaml
import matplotlib.pyplot as plt
from keras import losses
import numpy as np
import tensorflow as tf
import os
import csv

tf.reset_default_graph()
#number of epochs for training
num_epochs = 100
#number of possible labels
n_labels = 2
#number of features
n_features = 2
#size of each epoch (i.e. batch)
batch_size = 16
#number of hidden units in input layer lstm cell
#hidden layer number of units
#percentage to drop
dropout = 0.3

def read_data_sets(name):
    read_data_set = []
    with open(name, "r") as fault_data:
        for row in csv.reader(fault_data):
            read_data_set = np.append(read_data_set, np.array(row))
    return read_data_set

def generate_train_test_data_sets():
    # divides the entire data set into sequences of 10 data points
    #data = np.reshape(data_set, [int(len(data_set)/5), n_features+1])
    #print(data)
    #data = data[int((len(data)/20)+1):]
    #data = np.reshape(data, [int(len(data)/n_input), n_input, n_features+1])
    #print(len(data))
    #print(np.as_array(data_set.shape()))
    data = np.reshape(data_set, [int(len(data_set)/(n_features+1)), n_features+1])
    print(len(data))
    # shuffles the set of sequences
    np.random.shuffle(data)

    # splits the data set into features and labels
    seg_label_data = data[:, n_features]
    seg_feature_data = data[:, :n_features]

    return np.asarray(seg_feature_data), np.asarray(seg_label_data)

training_file = '../data/dosResults.csv'
data_set = read_data_sets(training_file)
print("Loaded training data...")
features, labels_norm = generate_train_test_data_sets()
print(len(features))
labels = []
# convert label data to one_hot arrays
for k in range(0, len(labels_norm)):
    one_hot_label = np.zeros([2], dtype=float)
    one_hot_label[int(float(labels_norm[k]))] = 1.0
    labels = np.append(labels, one_hot_label)
labels = np.reshape(labels, [-1, n_labels])

# Create LSTM
# expected input data shape: (batch_size, timesteps, data_dim)
model = Sequential()
model.add(Dense(2, activation='tanh', input_shape=(2,)))
model.add(Dropout(0.2))
model.add(Dense(4, activation='tanh'))
model.add(Dropout(0.2))
model.add(Dense(n_labels, activation='sigmoid'))


adam = Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=False)
model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

history = model.fit(np.reshape(features, (features.shape[0], n_features)),labels, batch_size=batch_size, epochs=num_epochs, validation_split= .3)

# summarize history for accuracy
plt.plot(history.history['acc'])
plt.plot(history.history['val_acc'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()
# summarize history for loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.show()

model_yaml = model.to_yaml()
with open("../config/model.yaml", "w") as yaml_file:
    yaml_file.write(model_yaml)
model.save_weights("../config/model.h5")
