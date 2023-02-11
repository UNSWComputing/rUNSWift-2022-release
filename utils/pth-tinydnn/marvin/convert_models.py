import torch
import sys
import os

# Downloads all weights (including batch normalisation parameters) into one file

def dump_layer_weights(f, param_type, param_values):  
    if "weight" in param_type or "mean" in param_type:
        for val in param_values.view(-1):
            f.write('%.24f ' % val.item())  
    elif "bias" in param_type or "var" in param_type:
        for val in param_values.view(-1):
            f.write('%.24f ' % val.item())
        f.write('\n') 
    

def dump_net_weights(model_path, save_path):
    ckpt = torch.load(model_path)
        
    with open(save_path, 'w') as f:
        for k, v in ckpt.items():
            dump_layer_weights(f, k, v)

def main():
    if len(sys.argv) != 3:
        print('python convert_models.py model_path save_path')
        sys.exit(1)

    model_path = sys.argv[1]
    save_path = sys.argv[2]

    if not os.path.exists(model_path) or os.path.isdir(model_path):
        print('[ERROR] Model file not exists!')
        sys.exit(2)
    
    dump_net_weights(model_path, save_path)

            
if __name__ == "__main__":
        main()
    
# Example call: python convert_models.py "model/model.pth" "weights/model.weights"