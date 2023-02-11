# Auto Optimizer

An auto hyperparameter optimizer based on hyperOpt to deal with parameter searching with black-box algorithms

## Prerequisites

Python3
hyperopt 
```bash
pip install hyperopt
```
numpy
```bash
pip install numpy
```

## Usage(localiser)

Store physical robot data in "./phy/"
Store Simulation data in "./simu/"

Rename input data as "in_{index}.txt"
Rename ground truth data as "pos_{index}.txt"

```bash
python3 localisationOptimizer _n_data _n_trials is_training is_physical
```

_n_data: int, number of data files
_n_trials: int, number of training trials
is_training: bool, run training or testing
is_physcal: {phy, simu}

