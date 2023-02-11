// adapted from https://github.com/tiny-dnn/tiny-dnn/blob/master/tiny_dnn/core/kernels/conv2d_op.h
/*
    Copyright (c) 2013, Taiga Nomi and the respective contributors
    All rights reserved.

    Use of this source code is governed by a BSD-style license that can be found
    in the LICENSE file.
*/
#pragma once

#include "tiny_dnn/core/framework/op_kernel.h"

#include "tiny_dnn/core/kernels/conv2d_op_avx.h"
#include "tiny_jnn/core/kernels/conv2d_op_internal.h"
#include "tiny_dnn/core/kernels/conv2d_op_nnpack.h"

namespace tiny_jnn {

template<int window_width_tiny_jnn, int window_height_tiny_jnn, int w_dilation_tiny_jnn, int h_dilation_tiny_jnn>
class Conv2dOp : public core::OpKernel {
 public:
  explicit Conv2dOp(const core::OpKernelConstruction &context)
    : core::OpKernel(context) {}

  void compute(core::OpKernelContext &context) override {
    auto params = OpKernel::params_->conv();

    // incomimg/outcoming data
    const tensor_t &in_data = context.input(0);
    const tensor_t &W       = context.input(1);
    const tensor_t &bias    = context.input(2);
    tensor_t &out_data      = context.output(0);

    // initialize outputs
    fill_tensor(out_data, tiny_dnn::float_t{0});

    // call convolution algorithm depending
    // on the selected engine type

    const core::backend_t engine = context.engine();

    if (engine == core::backend_t::internal) {
      kernels::conv2d_op_internal<window_width_tiny_jnn,
                                  window_height_tiny_jnn,
                                  w_dilation_tiny_jnn,
                                  h_dilation_tiny_jnn>(in_data, W[0], bias[0], out_data, params, context.parallelize());
    } else if (engine == core::backend_t::nnpack) {
      kernels::conv2d_op_nnpack(in_data, W[0], bias[0], out_data, params);
    } else if (engine == core::backend_t::avx) {
      kernels::conv2d_op_avx(in_data, W[0], bias[0], out_data, params,
                             context.parallelize());
    } else {
      throw nn_error("Not supported engine: " + to_string(engine));
    }
  }
};

}  // namespace tiny_jnn
