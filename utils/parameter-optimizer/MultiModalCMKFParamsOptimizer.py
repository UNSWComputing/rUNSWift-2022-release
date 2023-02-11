from hyperopt import fmin, tpe, hp, STATUS_OK, Trials, space_eval
import numpy as np
import os
import time
import sys
# import GPyOpt


# For TPE
search_space_MMKF = {
    # Search Space, uniform distribution for now
    "ModeSplitWeightMultiplyFactor": hp.uniform("ModeSplitWeightMultiplyFactor", 0, 1),
    "OdometryForwardMultiplyFactor": hp.uniform("OdometryForwardMultiplyFactor", 1, 50),
    "OdometryLeftMultiplyFactor": hp.uniform("OdometryLeftMultiplyFactor", 1, 50),
    "OdometryHeadingMultiplyFactor": hp.uniform("OdometryHeadingMultiplyFactor", 1, 50),
    "AngleUncertainty" : hp.uniform("AngleUncertainty", 5, 50),
    "UpdateHeadingUncertainty": hp.uniform("UpdateHeadingUncertainty", 5, 50),
    "SimilarHeadingThresh": hp.uniform("SimilarHeadingThresh", 5, 50),
    "SimilarXThresh": hp.uniform("SimilarXThresh", 100, 1000),
    "SimilarYThresh": hp.uniform("SimilarYThresh", 100, 1000),
    "MinCMKFWeight": hp.uniform("MinCMKFWeight", 0.001, 0.1),
}

# For Beyasian
bounds = [
    {'name': 'ModeSplitWeightMultiplyFactor', 'type': 'continuous', 'domain': (0,1)},
    {'name': 'OdometryForwardMultiplyFactor', 'type': 'continuous', 'domain': (1,50)},
    {'name': 'OdometryLeftMultiplyFactor', 'type': 'continuous', 'domain': (1,50)},
    {'name': 'OdometryHeadingMultiplyFactor', 'type': 'continuous', 'domain': (1,50)},
    {'name': 'AngleUncertainty', 'type': 'continuous', 'domain': (5,50)},
    {'name': 'UpdateHeadingUncertainty', 'type': 'continuous', 'domain': (5,50)},
    {'name': 'SimilarHeadingThresh', 'type': 'continuous', 'domain': (5,50)},
    {'name': 'SimilarXThresh', 'type': 'continuous', 'domain': (100, 1000)},
    {'name': 'SimilarYThresh', 'type': 'continuous', 'domain': (100, 1000)},
    {'name': 'MinCMKFWeight', 'type': 'continuous', 'domain': (0.001, 0.1)}
]

global n_data
RUNSWIFT_DIR = os.environ['RUNSWIFT_CHECKOUT_DIR']
cfg_path = RUNSWIFT_DIR +  "/image/home/nao/data/MultiModalCMKFParams.cfg"

global data_dir
data_dir = "phy/"

def preprocess(output_dir, ground_truth_dir):
    try:
        output_file = open(output_dir)
        ground_truth_file = open(ground_truth_dir)
    except:
        return None

    output = []
    for line in output_file:
        x, y, _, ts = line.split()
        output.append((float(x), float(y), int(ts)))

    ground_truth = []
    for line in ground_truth_file:
        x, y, _, ts = line.split()
        ground_truth.append((float(x), float(y), int(ts)))
    
    output_file.close()
    ground_truth_file.close()
    return output, ground_truth

def align_data(output, ground_truth):
    processed_data = []
    _cur_line = 0
    _length_output = len(output)
    _length_gt = len(ground_truth)
    _cur_line = 0
    for line in output:
        _cur_line += 1
        if _cur_line >= 10000:
            return processed_data
        x_out, y_out, ts_out = line
        for i in range(_cur_line, _length_gt):
            x_gt, y_gt, ts_gt = ground_truth[i]
            if ts_gt > ts_out:
                _cur_line = i
                break

        if _cur_line >= _length_gt - 1:
            return processed_data
        
        x_gt, y_gt, ts_gt = ground_truth[_cur_line-1]
        processed_data.append((x_gt, y_gt, x_out, y_out))
    return processed_data


def get_cost(processed_data):
    # Using Euclid distance with Mean Square Error
    lines = len(processed_data)
    MSE = 0
    for i in range(lines):
        MSE += get_L2_distance_2(processed_data[i][0], processed_data[i][1], processed_data[i][2], processed_data[i][3])

    MSE /= lines
    return MSE


def get_L2_distance_2(x1, y1, x2, y2):

    distance = (x1 - x2) ** 2 + (y1 - y2) ** 2
    return distance

def run_simulation(params, path, n=1):
    # change cfg
    # run simulation X times with cross validation
    # get the mean of the sum of the MSEs
    if params is not None:
        cfg_file = open(path, 'w')
        for item in params.items():
            cfg_file.writelines(item[0]+"="+str(item[1])+"\n")
        cfg_file.close()
    # os.system("rm -rf out_1.txt")
    # os.system("state-estimation-simulator 1.txt  out_1.txt")
    # time.sleep(1)
    for i in range(1, n+1):
        _out_file = data_dir + "out_" + str(i) + ".txt"
        _in_file = data_dir + "in_" + str(i) + ".txt"
        _command = "state-estimation-simulator " + _in_file + " " + _out_file 
        os.system(_command)
        # print("Executed: " + str(i))
    


def f(params):
    # output,ground_truth = preprocess("out_1.txt", "gt_1.txt")
    # aligned_data = align_data(output, ground_truth)
    # loss = get_cost(aligned_data)
    
    
    
    run_simulation(params, cfg_path, n_data)
    loss = data_batch(5)
    return {'loss': loss, 'status': STATUS_OK}
    # return loss

def data_batch(n = 1):
    loss = 0
    for i in range(1, n+1):
        _out_file = data_dir + "out_" + str(i) + ".txt"
        _gt_file = data_dir + "pos_" + str(i) + ".txt"

        _out, _gt = preprocess(_out_file, _gt_file)
        _aligned = align_data(_out, _gt)
        _loss = get_cost(_aligned)
        loss += _loss
    return int(loss/n)

def get_best_trial(best_trial):
    for_eval = {}
    for k, v in best_trial.items():
        if len(v) == 0:
            for_eval[k] = None
        else:
            for_eval[k] = v[0]
    return space_eval(search_space_MMKF, for_eval)



if __name__ == "__main__":

    args = sys.argv
    if len(args) is not 5:
        sys.exit()
    _, _n_data, n_trials, is_training, is_physical = args
    n_data = int(_n_data)

    run_simulation(None, cfg_path, n_data)

    if is_physical is "simu":
        data_dir = "simu"

    if is_training == 'T' or is_training == 't':
        t = Trials()
        best = fmin(f, search_space_MMKF, algo=tpe.suggest, max_evals=int(n_trials), trials = t)
        best_param= get_best_trial(t.best_trial['misc']['vals'])
        print(best_param)

        run_simulation(best_param, cfg_path, n_data)
    print(data_batch(n_data))