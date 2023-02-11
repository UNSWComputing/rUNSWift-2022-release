#pragma once

#include "tiny_dnn/layers/layer.h"

namespace tiny_jnn {
  class transpose_layer : public tiny_dnn::layer {
    using shape3d = tiny_dnn::shape3d;
    using tensor_t = tiny_dnn::tensor_t;
    using vector_type = tiny_dnn::vector_type;
    using vec_t = tiny_dnn::vec_t;

  public:
    /**
     * Construct a transpose layer with specified width, height and channels.
     * This constructor is suitable for adding a transpose layer after spatial
     * layers such as convolution / pooling layers.
     *
     * @param in_width    [in] number of input elements along width
     * @param in_height   [in] number of input elements along height
     * @param in_channels [in] number of channels (input elements along depth)
     */
    transpose_layer(size_t in_width, size_t in_height, size_t in_channels)
        : transpose_layer(shape3d(in_width, in_height, in_channels)) {}

    /**
     * Construct a transpose layer with specified input shape.
     *
     * @param in_shape [in] shape of input tensor
     */
    explicit transpose_layer(const shape3d &in_shape)
        : layer({vector_type::data}, {vector_type::data}) {
      set_in_shape(in_shape);
    }

    /**
     * @param in_data      input vectors of this layer (data, weight, bias)
     * @param out_data     output vectors
     **/
    void forward_propagation(const std::vector<tensor_t *> &in_data,
                             std::vector<tensor_t *> &out_data) override {
      const tensor_t &x     = *in_data[0];
      tensor_t       &y     = *out_data[0];
      // tensor_t is a vector of vectors of floats
      // outer vector is for batching, inner vector is [depth][height][width]
      for (size_t    sample = 0; sample < x.size(); ++sample) {
        const vec_t &in = x[sample];
        vec_t       &a  = y[sample];
        for_i(in_shape_.depth_, [&](size_t d) {
          for_i(in_shape_.height_, [&](size_t h) {
            for_i(in_shape_.width_, [&](size_t w) {
              a[out_shape_.get_index(h, w, d)] = in[in_shape_.get_index(w, h, d)];
            });
          });
        });
      }
    }

    /**
     * return delta of previous layer (delta=\frac{dE}{da}, a=wx in
     *fully-connected layer)
     * @param in_data  input vectors (same vectors as forward_propagation)
     * @param out_data output vectors (same vectors as forward_propagation)
     * @param out_grad gradient of output vectors (i-th vector correspond with
     *out_data[i])
     * @param in_grad  gradient of input vectors (i-th vector correspond with
     *in_data[i])
     **/
    virtual void back_propagation(const std::vector<tensor_t *> &in_data,
                                  const std::vector<tensor_t *> &out_data,
                                  std::vector<tensor_t *> &out_grad,
                                  std::vector<tensor_t *> &in_grad) override {};

    std::vector<shape3d> in_shape() const override { return {in_shape_}; }

    std::vector<shape3d> out_shape() const override {
      return {out_shape_};
    }

    void set_in_shape(const shape3d &in_shape) override {
      this->in_shape_  = in_shape;
      this->out_shape_ = shape3d(in_shape_.height_, in_shape_.width_, in_shape_.depth_);
    }

    std::string layer_type() const override { return "transpose"; }

  private:
    shape3d in_shape_, out_shape_;
  };
}  // namespace tiny_jnn
