import pandas as pd
import numpy as np

class DataLoader:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = pd.read_csv(self.file_path)
        self.train_data, self.test_data = self.split_data()

    def split_data(self, train_size=0.8):
        train_data = self.data.sample(frac=train_size, random_state=42)
        test_data = self.data.drop(train_data.index)
        return train_data, test_data
    def layer