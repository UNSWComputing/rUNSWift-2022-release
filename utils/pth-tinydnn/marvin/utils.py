import torch
import time
import numpy as np
import matplotlib.pyplot as plt
from postprocess import *
from thop import profile
from PIL import Image

'''Trains the model and returns the average training loss of epoch'''
def train(model, device, trainloader, criterion, optimizer, epoch):
    model.train() # set to training mode
    
    running_loss = 0
    
    for batch_idx, (data, label) in enumerate(trainloader):
        data, label = data.to(device), label.to(device)

        optimizer.zero_grad()
        output = model(data)
               
        loss = criterion(output, label)
        
        loss.backward()
        optimizer.step()
        
        running_loss += loss.item()
    
    avg_loss = running_loss / len(trainloader.dataset)
    print("Epoch", epoch + 1)
    print("Training loss: %.5f" % (avg_loss))
    
    return avg_loss


'''Evaluates the model on the validation set'''    
def evaluate(model, device, valloader, criterion):
    model.eval()
    
    running_loss = 0
    
    with torch.no_grad():
        # Calculate average loss
        for batch_idx, (data, label) in enumerate(valloader):
            data, label = data.to(device), label.to(device)
            
            output = model(data)
        
            loss = criterion(output, label)
            running_loss += loss.item()
        
        avg_loss = running_loss / len(valloader.dataset)
        print("Validation loss: %.5f" % (avg_loss))
  
        # Calculate average inference time (excludes time cost of computing loss)
        t0 = time.time()
        for batch_idx, (data, label) in enumerate(valloader):
            data = data.to(device)
            
            output = model(data)
        
        avg_time = 1000 * (time.time() - t0) / len(valloader.dataset)
        print("Average inference time: %.3f ms" % (avg_time))
    
    return avg_loss, avg_time


'''Plot the bounding boxes and save the results'''        
def display_results(model, device, valloader, thresh=None, save_dst=None):

    rows = len(valloader.dataset)
    
    fig = plt.figure(figsize=(16, rows * 4))
    axes = []
    
    i = 1

    with torch.no_grad():
        # Iterate over batches
        for batch_idx, (data, label) in enumerate(valloader):
            data, label = data.to(device), label.to(device)
            output = model(data)
            
            # Just want region prediction output
            if type(output) == tuple:
                output = output[1]
            
            # Iterate over batch
            for data_single, label_single, output_single in zip(data, label, output):
                output_single = torch.sigmoid(output_single) # range between 0 and 1
                output_single = np.array(torch.Tensor.cpu(output_single)[0])
                
                data_single = np.array(torch.Tensor.cpu(data_single)[0])
                min_val = np.min(data_single)
                max_val = np.max(data_single)
                data_single = (data_single - min_val) / ((max_val - min_val))
                
                label_single = np.array(torch.Tensor.cpu(label_single)[0], dtype=np.uint8)
                
                # Include bounding boxes
                if thresh:
                    #t0 = time.time()
                    predicted_bboxes, data_single = bbox_predictions(data_single, output_single, thresh)
                    #print(f'Post-processing: {(1000 * (time.time() - t0)):.3f} ms')
                   
                # Convert predicted labels to binary
                if thresh is not None:
                    output_single = output_single > thresh
                    
                # Plot input image, predicted labels, labels
                combined = np.hstack([label_single, data_single, output_single])
                axes.append(fig.add_subplot(rows, 1, i, title=f'{i}: Actual Labels / Image / Predicted Labels'))
                plt.imshow(combined, cmap='gray')
                
                i += 1

            # Remove ticks
            for ax in axes:
                ax.set_xticks([])
                ax.set_yticks([])      
    
    # Save figure
    if save_dst is not None:
        plt.savefig(save_dst)
    
    # Plot figure
    plt.show()


'''Save predicted regions as separate images'''
def save_results(model, device, valloader, thresh=None, save_dst=None):

    rows = len(valloader.dataset)
    counter = 0

    with torch.no_grad():
        # Iterate over batches
        for batch_idx, (data, label) in enumerate(valloader):
            data, label = data.to(device), label.to(device)
            output = model(data)
            
            # Just want region prediction output
            if type(output) == tuple:
                output = output[1]
            
            # Iterate over batch
            for data_single, label_single, output_single in zip(data, label, output):
                output_single = torch.sigmoid(output_single) # range between 0 and 1
                output_single = np.array(torch.Tensor.cpu(output_single)[0])
                
                data_single = np.array(torch.Tensor.cpu(data_single)[0])
                min_val = np.min(data_single)
                max_val = np.max(data_single)
                data_single = (data_single - min_val) / ((max_val - min_val))
                
                label_single = np.array(torch.Tensor.cpu(label_single)[0], dtype=np.uint8)
                
                   
                # Convert predicted labels to binary
                if thresh is not None:
                    output_single = output_single > thresh
                    
                # Save image
                img = Image.fromarray(output_single)
                img.save(save_dst + str(counter) +'.png')
                counter += 1


'''Count the number of flops and parameters and return them'''
def count_params(model, input):
    flops, params = profile(model, inputs=(input, ), verbose=False)
    print(f'params:\t{int(params):,}')
    print(f'flops:\t{flops / 1000000:.3f}M')
    return flops, params