import os
import sys

PWD = os.path.dirname(__file__)
RUNSWIFT_CHECKOUT_DIR = os.path.join(PWD, '..', '..')
RUNSWIFT_CHECKOUT_DIR = os.path.realpath(RUNSWIFT_CHECKOUT_DIR)
VENVS_DIR = os.path.join(RUNSWIFT_CHECKOUT_DIR, '.virtualenvs')
VENV_DIR = os.path.join(RUNSWIFT_CHECKOUT_DIR, '.virtualenvs', 'pth-tinydnn')


def setup_torch():
    if not os.path.isdir(VENV_DIR):
        os.makedirs(VENVS_DIR, exist_ok=True)
        # venv (not virtualenv) doesn't have activate_this.py and running
        # venv/bin/python doesn't indicate it's running the right python
        os.system(f'python3 -m virtualenv {VENV_DIR}')

    if 'VIRTUAL_ENV' not in os.environ or os.environ['VIRTUAL_ENV'] != VENV_DIR:
        exec(open(os.path.join(VENV_DIR, 'bin', 'activate_this.py')).read(),
             {'__file__': os.path.join(VENV_DIR, 'bin', 'activate_this.py')})
        os.execvp('python3', ['python3'] + sys.argv)
        # os.execlp('python3', 'python3', *sys.argv)

    try:
        import torch
    except ImportError:
        os.system(
            'pip install torch==1.5.0+cpu torchvision==0.6.0+cpu -f https://download.pytorch.org/whl/torch_stable.html')
        import torch
