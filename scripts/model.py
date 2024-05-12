# General and visualization imports
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

# PyTorch imports
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
from torchvision import datasets
from torchvision.transforms import v2
from torch.utils.data import DataLoader
import deeplake

# Model parameters
class CNN(nn.Module):
    def __init__(self):
        super(CNN, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, kernel_size=5, padding=2)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=5, padding=2)
        self.pool = nn.MaxPool2d(2, 2)
        self.fc1 = nn.Linear(64 * 7 * 7, 128)
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, 26)  # 26 classes for letters A-Z + 1 for undetermined (necessary)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))
        x = self.pool(torch.relu(self.conv2(x)))
        x = x.view(-1, 64 * 7 * 7)
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)

        return x



if __name__ == "__main__":
    
  # Transform to preprocess the data
  transform = v2.Compose([
      v2.Grayscale(),
      v2.Resize((28, 28)),
      v2.Lambda(lambda x: v2.functional.invert(x)),
      v2.ToImage(), # used to be totensor, deprecated
      v2.ToDtype( torch.float32, scale=True),
      v2.Normalize((0.1307,), (0.3081,)) 
  ])
  
  # Deeplake transform
  training_transform = v2.Compose([
      v2.ToPILImage(), # Just based on deeplake guidance w PyTorch
      v2.ToImage(),
      v2.ToDtype(torch.float32, scale=True),
      v2.Normalize((0.1307,), (0.3081,))
  ])
  
  # Deeplake datasets
  train_ds = deeplake.load("hub://activeloop/emnist-letters-train")
  test_ds = deeplake.load("hub://activeloop/emnist-letters-test")
  
  # Dataloaders
  train_dataloader= train_ds.pytorch(batch_size = 64, num_workers = 2, 
      transform = {'images': training_transform, 'labels': None}, 
      shuffle = False)
  
  test_dataloader= train_ds.pytorch(batch_size = 64, num_workers = 2, 
      transform = {'images': training_transform, 'labels': None}, 
      shuffle = False)

  
  model = CNN()

  
  # Loss Function
  lossf = nn.CrossEntropyLoss()
  optimizer = optim.Adam(model.parameters(), lr=0.001)
  
  # Training
  num_epochs = 10
  for epoch in range(num_epochs):
      running_loss = 0.0
      for i, data in enumerate(train_dataloader, 0):
          inputs, labels = data
          labels = torch.flatten(labels) - 1
          #print(inputs.shape)
          #print(labels)      
          optimizer.zero_grad()
          outputs = model(inputs)
          loss = lossf(outputs, labels)
          loss.backward()
          optimizer.step()
          running_loss += loss.item()
          if (i+1) % 100 == 0:
              print(f'Epoch [{ epoch + 1 }/{num_epochs}], Step [{ i + 1 }/{ len(train_dataloader) }], Loss: { running_loss/100:.4f }')
              running_loss = 0.0

  
  # Testing
  correct = 0
  total = 0
  with torch.no_grad():
      for data in test_dataloader:
          inputs, labels = data
          labels = torch.flatten(labels) - 1
          outputs = model(inputs)
          _, predicted = torch.max(outputs.data, 1)
          total += labels.size(0)
          correct += (predicted == labels).sum().item()
  
  print(f'Accuracy on test set: {100 * correct / total:.2f}%')
  
  
  torch.save(model, "letter_model.pt")

