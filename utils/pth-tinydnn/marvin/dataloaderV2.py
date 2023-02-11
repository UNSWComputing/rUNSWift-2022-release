import os
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils import data
import csv

class RobotAEDataset(data.Dataset):

    def __init__(self, root_dir, downsample_factor=0.25, device="cuda"): # added downsample_factor as an arg
        self.root_path = root_dir # path to data
        self.downsample_factor = downsample_factor
        self.device = device
        self.data, self.labels = self.load_image()


    def __len__(self):
        return self.data.size()[0]


    def __getitem__(self, idx):
        sample = [self.data[idx], self.labels[idx]]  # (data, label)
        return sample


    def load_csv(self): # load 'data.csv' into a list
        name_list = []
        csv_path = os.path.join(self.root_path, "data.csv")
        
        with open(csv_path, 'r') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                name_list.extend(row)

        return name_list


    def load_image(self):
        imgs = []
        labels = []

        name_list = self.load_csv()

        data_path = os.path.join(self.root_path, 'png_complete')
        label_path = os.path.join(self.root_path, 'labelled_png_complete')

        for file in name_list:
            data_file_path = os.path.join(data_path, file)
            data = cv2.imread(data_file_path, cv2.IMREAD_GRAYSCALE) # read in image as grayscale
            data = cv2.resize(data, (0,0), fx=self.downsample_factor, fy=self.downsample_factor) # resize image
            data = data[np.newaxis, ...]  # from H, W to 1, H, W
            imgs.append(data)

            label_file_path = os.path.join(label_path, file)
            label = cv2.imread(label_file_path, cv2.IMREAD_GRAYSCALE) # read in label as grayscale
            label = cv2.resize(label, (0,0), fx=self.downsample_factor, fy=self.downsample_factor) # resize label
            label //= 255 # make label binary (0s and 1s)
            label = label[np.newaxis, ...]  # from H, W to 1, H, W
            labels.append(label)

        imgs = torch.from_numpy(np.array(imgs)).float()
        labels = torch.from_numpy(np.array(labels)).float()
        return imgs, labels # return two tensors of shape (-1, 1, H, W)


if __name__ == "__main__":

    train_dataset = RobotAEDataset('../data/training/', downsample_factor=0.25, device="cuda")
    val_dataset = RobotAEDataset('../data/validation/', downsample_factor=0.25, device="cuda")
    print(len(train_dataset), 'training examples.')
    print(len(val_dataset), 'test examples.')
    print('Train dataset shape:', *train_dataset.data.size())
    print('Test dataset shape:', *val_dataset.data.size())
