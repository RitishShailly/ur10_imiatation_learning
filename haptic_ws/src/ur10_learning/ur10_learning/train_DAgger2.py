import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim
import pickle
import numpy as np
import os
import sys

class HumanData(Dataset):
    def __init__(self, filename):
        with open(filename, "rb") as file:
            self.data = pickle.load(file)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.FloatTensor(self.data[idx])

class BC(nn.Module):
    def __init__(self, hidden_dim):
        super(BC, self).__init__()
        self.state_dim = 18        
        self.action_dim = 6
        self.linear1 = nn.Linear(self.state_dim, hidden_dim)
        self.linear2 = nn.Linear(hidden_dim, hidden_dim)
        self.linear3 = nn.Linear(hidden_dim, self.action_dim)
        self.loss_func = nn.MSELoss()

    def encoder(self, state):
        h1 = torch.tanh(self.linear1(state))
        h2 = torch.tanh(self.linear2(h1))
        return self.linear3(h2)

    def forward(self, x):
        state = x[:, :self.state_dim]
        a_target = x[:, -self.action_dim:]
        a_predicted = self.encoder(state)
        loss = self.loss(a_predicted, a_target)
        return loss

    def loss(self, a_predicted, a_target):
        return self.loss_func(a_predicted, a_target)

def main():
    # Define learning parameters
    EPOCH = 1000
    LR = 0.001
    LR_STEP_SIZE = 1000
    LR_GAMMA = 0.1
    n_models = 5
    BATCH_SIZE_TRAIN = 200

    # Check CUDA availability
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # Load data
    segment_name = "segment123"
    # foldername = "~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/segments/models/"+segment_name
    foldername = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/env1/processed_DAgger_data/'
    foldername = os.path.expanduser(foldername)

    save_foldername = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/env1/DAgger_model2/'
    save_foldername = os.path.expanduser(save_foldername)

    # filename = os.path.join(foldername, 'pdata', 'sa_pairs.pkl')
    filename = os.path.join(foldername,'sa_pairs_with_positions.pkl')

    train_data = HumanData(filename)
    train_set = DataLoader(dataset=train_data, batch_size=BATCH_SIZE_TRAIN, shuffle=True)

    for n in range(n_models):
        print(f'[*] Training model {n+1}')
        model = BC(64).to(device)
        optimizer = optim.Adam(model.parameters(), lr=LR)
        scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=LR_STEP_SIZE, gamma=LR_GAMMA)

        for epoch in range(EPOCH):
            for batch, x in enumerate(train_set):
                x = x.to(device)
                optimizer.zero_grad()
                loss = model(x)
                loss.backward()
                optimizer.step()
            scheduler.step()
            if epoch % 100 == 0:
                print(epoch, loss.item())

        torch.save(model.state_dict(), os.path.join(save_foldername, f"model{n+1}"))

if __name__ == "__main__":
    main()
