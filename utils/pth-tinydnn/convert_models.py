#!/usr/bin/python3
# -*- coding: utf-8 -*-
# adapted from https://github.com/tiny-dnn/tiny-dnn/blob/master/examples/ssd_detection/convert_models.py

import collections
import os
import re
import sys
from typing import Dict

import utils

utils.setup_torch()

import torch
import marvin.basic_models

nets = {
    1: ["vgg.0", "vgg.2", "vgg.5", "vgg.7", "vgg.10", "vgg.12", "vgg.14", "vgg.17", "vgg.19", "vgg.21"],
    2: ["vgg.24", "vgg.26", "vgg.28", "vgg.31", "vgg.33"],
    3: ["extras.0", "extras.1"],
    4: ["extras.2", "extras.3"],
    5: ["extras.4", "extras.5"],
    6: ["extras.6", "extras.7"],
    7: ["loc.0"],
    8: ["loc.1"],
    9: ["loc.2"],
    10: ["loc.3"],
    11: ["loc.4"],
    12: ["loc.5"],
    13: ["conf.0"],
    14: ["conf.1"],
    15: ["conf.2"],
    16: ["conf.3"],
    17: ["conf.4"],
    18: ["conf.5"],
}


def dump_layer_weights(f, weight, bias):
    # [num_out_channels, num_in_channels, window_height, window_width] = weight.shape
    for i in range(weight.shape[0]):
        for j in range(weight.shape[1]):
            for k in range(weight.shape[2]):
                for m in range(weight.shape[3]):
                    f.write('%.24f ' % weight[i][j][k][m])
    _ = f.write('\n')
    for i in range(bias.shape[0]):
        f.write('%.24f ' % bias[i])
    _ = f.write('\n')


def dump_nets(f, class_path, nets_py):
    def get_net_cpp(net_id, named_layers_py):
        out_height, out_width = 240, 320
        out_channels = 1

        def get_layer_cpp(layer_name: str, layer_py):
            nonlocal out_height, out_width
            nonlocal out_channels
            if isinstance(layer_py, torch.nn.Conv2d):
                in_height, in_width = out_height, out_width
                h_dilation, w_dilation = layer_py.dilation
                window_height, window_width = layer_py.kernel_size
                in_channels = layer_py.in_channels
                out_channels = layer_py.out_channels
                h_stride, w_stride = layer_py.stride
                out_height, out_width = in_height // h_stride, in_width // w_stride
                assert out_height == (in_height + 2 * layer_py.padding[0] - window_height) // h_stride + 1, \
                    f'only padding::same is supported: {layer_py}'
                assert out_width == (in_width + 2 * layer_py.padding[1] - window_width) // w_stride + 1, \
                    f'only padding::same is supported: {layer_py}'
                return f"<< conv<{window_width}, {window_height}, {w_dilation}, {h_dilation}>({in_width}, {in_height}, {window_width}, {window_height}, {in_channels}, {out_channels}, tiny_dnn::padding::same, true, {w_stride}, {h_stride}, {w_dilation}, {h_dilation})"
            elif isinstance(layer_py, torch.nn.BatchNorm2d):
                return f"<< batch_norm({out_width} * {out_height}, {layer_py.num_features}, {layer_py.eps}, {layer_py.momentum}, net_phase::test)"
            elif isinstance(layer_py, torch.nn.ReLU):
                return f"<< relu({out_width}, {out_height}, {out_channels})"
            elif isinstance(layer_py, torch.nn.Upsample):
                return f"// << {layer_py}\n"
            elif isinstance(layer_py, marvin.basic_models.transpose_layer):
                layer_cpp = f" << transpose({out_width}, {out_height}, {out_channels})\n"
                out_height, out_width = out_width, out_height
                return layer_cpp
            else:
                raise NotImplementedError(f"can't convert {layer_name} to cpp: {layer_py}")

        layer_cpps = [get_layer_cpp(layer_name, layer_py) for layer_name, layer_py in named_layers_py]
        nn = '\n'.join(layer_cpps)
        return f'''
  tiny_dnn::network<tiny_dnn::sequential> nn{net_id};
  nn{net_id} {nn};
  nets.push_back(nn{net_id});
'''

    # like torch.nn.module.apply, but with names
    def named_apply(model, fun, names=[]):
        for child_name, child in model.named_children():
            child_names = names + [child_name]
            fun('.'.join(child_names), child)
            named_apply(child, fun, child_names)

    class_path = class_path.split('.')
    package_name, module_name, class_name = '.'.join(class_path[:-1]), class_path[-2], class_path[-1]
    package = __import__(package_name)
    module = getattr(package, module_name)
    model: torch.nn.Module = getattr(module, class_name)()

    with open('test-template.cpp', 'r') as template_file:
        data = template_file.read()
        nets_cpp = []
        assert len(nets_py) == 1, "can't handle more than 1 net, yet"
        for net_id in nets_py:
            named_descendants = []
            named_apply(model, lambda name, descendant: named_descendants.append((name, descendant)))
            named_layers_py = [(name, descendant) for name, descendant in named_descendants if
                               not next(descendant.children(), False)]
            nets_cpp.append(get_net_cpp(net_id, named_layers_py))
        f.write(data % {'nets': ''.join(nets_cpp)})


def dump_net_weights(model_path, output_folder, class_path=None):
    global nets
    ckpt: Dict = torch.load(model_path, map_location=torch.device('cpu'))

    if class_path is not None:
        # attempt at auto-creating `nets` - works for our .pth but not ssd300_mAP_77.43_v2.pth
        nets = collections.defaultdict(list)
        key: str
        for key, value in ckpt.items():
            if key.endswith('.weight') and re.sub('\\.weight$', '.bias', key) in ckpt and len(value.shape) == 4:
                # net_id = int(key.split('.')[1])
                net_id = 1
                nets[net_id].append(key[:-len('.weight')])
        output_cpp_file_path = os.path.join(output_folder, 'test.cpp')

        print('Saving cpp to %s' % output_cpp_file_path)
        with open(output_cpp_file_path, 'w') as cf:
            dump_nets(cf, class_path, nets)

    for net_id in nets:
        layers = nets[net_id]
        output_file_path = os.path.join(output_folder, '%02d.weights' % net_id)

        print('Saving weights to %s' % output_file_path)
        with open(output_file_path, 'w') as f:
            for layer in layers:
                weight = ckpt['%s.weight' % layer]
                bias = ckpt['%s.bias' % layer]
                dump_layer_weights(f, weight, bias)


def main():
    if not (3 <= len(sys.argv) <= 4):
        print('python convert_models.py model_path output_folder [package.module.model_class]')
        sys.exit(1)

    model_path = sys.argv[1]
    output_folder = sys.argv[2]
    class_path = sys.argv[3] if len(sys.argv) >= 4 else None
    if not os.path.exists(model_path) or os.path.isdir(model_path):
        print('[ERROR] Model file not exists!')
        sys.exit(2)
    os.makedirs(output_folder, exist_ok=True)

    dump_net_weights(model_path, output_folder, class_path)


if __name__ == '__main__':
    main()
