// adapted from https://github.com/tiny-dnn/tiny-dnn/blob/master/tiny_dnn/core/kernels/conv2d_op_internal.h
/*
    Copyright (c) 2013, Taiga Nomi and the respective contributors
    All rights reserved.

    Use of this source code is governed by a BSD-style license that can be found
    in the LICENSE file.
*/
#pragma once

#ifndef CNN_USE_SSE
#error enable CNN_USE_SSE please
#endif

// adapted from https://github.com/vectorclass/version2/blob/master/vectorf128.h
// Partial load. Load n elements and set the rest to 0
__m128 load_partial(int n, float const * p) {
  __m128 t1, t2, xmm;
  switch (n) {
    case 1:
      xmm = _mm_load_ss(p); break;
    case 2:
      xmm = _mm_castpd_ps(_mm_load_sd((double const*)p)); break;
    case 3:
      t1 = _mm_castpd_ps(_mm_load_sd((double const*)p));
      t2 = _mm_load_ss(p + 2);
      xmm = _mm_movelh_ps(t1, t2); break;
    case 4:
      xmm = _mm_loadu_ps(p); break;
    default:
      xmm = _mm_setzero_ps();
  }
  return xmm;
}

// add the first n elements of a.  this is slightly faster than horizontal_add(a) for n == 3, on a v6.
static inline float horizontal_add(__m128 const a, int n) {
  float p[4];
  _mm_storeu_ps(p, a);
  float sum = 0;
  for (int i = 0; i < n; ++i) {
    sum += p[i];
  }
  return sum;
}

// adapted from https://github.com/vectorclass/version2/blob/master/vectorf128.h
// Horizontal add: Calculates the sum of all vector elements.
static inline float horizontal_add(__m128 const a) {
  __m128 t1 = _mm_movehl_ps(a, a);
  __m128 t2 = _mm_add_ps(a, t1);
  __m128 t3 = _mm_shuffle_ps(t2, t2, 1);
  __m128 t4 = _mm_add_ss(t2, t3);
  return _mm_cvtss_f32(t4);
}

namespace tiny_jnn {
  using namespace tiny_dnn;
namespace kernels {
  using namespace tiny_dnn::kernels;

template<int window_width_tiny_jnn, int window_height_tiny_jnn, int w_dilation_tiny_jnn, int h_dilation_tiny_jnn>
inline void conv2d_op_internal(const tensor_t &in_data,
                               const vec_t &W,
                               const vec_t &bias,
                               tensor_t &out_data,
                               const core::conv_params &params,
                               const bool parallelize) {
  for_(parallelize, 0u, params.out.depth_,
       [&](const blocked_range &r) {
         size_t out_area    = params.out.area();
         size_t iw          = params.in_padded.width_;
         size_t id          = params.in.depth_;
         size_t ow          = params.out.width_;
         size_t oh          = params.out.height_;
         size_t od          = params.out.depth_;
         size_t kw          = params.weight.width_;
         size_t kh          = params.weight.height_;
         size_t w_dilation  = params.w_dilation;
         size_t h_dilation  = params.h_dilation;
         size_t elem_stride = params.w_stride;
         size_t line_stride = iw * params.h_stride;
         for (size_t sample = 0; sample < in_data.size(); sample++) {
           const vec_t &in = in_data[sample];
           vec_t &a        = out_data[sample];
           for (size_t o = r.begin(); o < r.end(); o++) {
             tiny_dnn::float_t *pa = &a[params.out.get_index(0, 0, o)];
             tiny_dnn::float_t pa_vec[oh * ow * 4];
             memset(pa_vec, 0, sizeof(pa_vec));
             for (size_t inc = 0; inc < id; inc++) {
               if (!params.tbl.is_connected(o, inc)) continue;
               size_t idx;
               idx                = params.weight.get_index(0, 0, id * o + inc);
               const tiny_dnn::float_t *pw  = &W[idx];
               idx                = params.in_padded.get_index(0, 0, inc);
               const tiny_dnn::float_t *pin = &in[idx];
               tiny_dnn::float_t *pout      = pa;
               tiny_dnn::float_t *pout_vec  = pa_vec;

               // if we don't use assembly, gcc knows to put this outside the y < oh loop, but we do, so gcc doesn't
               __m128 pw_element_vec[window_height_tiny_jnn];
               for (size_t wy = 0; wy < window_height_tiny_jnn; wy++) {
                 // we could do a load of 4 bytes and overrun the kernel array, since we do a
                 // horizontal add of only 3 element later, but the performance impact is minimal
                 pw_element_vec[wy] = load_partial(window_width_tiny_jnn, &pw[wy * window_width_tiny_jnn]);
               }
               for (size_t y = 0; y < oh; y++) {
                 const tiny_dnn::float_t *pin_line = pin;
                 if (window_width_tiny_jnn == 3 && window_height_tiny_jnn == 3 && w_dilation_tiny_jnn == 1) {
                   /*
                    * we have a lot of these 3*h kernels so let's try to optimize for them.  vectorization on Atom
                    * E3845 happens 4 32-bit floats at a time, so we are going to try something like this:
                    *
                    * pppp pppp pppp pppp pp (aligned, should be able to vectorize quite a bit)
                    * pp pppp pppp pppp pppp (unaligned, should still be able to vectorize quite a bit)
                    *
                    * kkk  (aligned, should vectorize)
                    *  kkk (aligned, should vectorize)
                    *   kk k (unaligned, can still vectorize on E3845 but should not on Z530)
                    *    k kk (unaligned, can still vectorize on E3845 but should not on Z530)
                    *
                    * MOVAPS/load_a (aligned) is the same speed as
                    * MOVUPS/load (unaligned) on E3845 but faster on Z530
                    *
                    * 32-bit only has 8 registers, 64-bit has 16.  If the outer loop goes across the output row, we
                    * use h registers just to store the kernel.  That's 3 of 8 in the common case.  If the outer loop
                    * goes across the kernel row, we use just 1 register and can use the other registers for the
                    * input and output, as the cpu can perform the dot product while waiting for next input to load.
                    *
                    * registers (worst case):
                    * - pw_element_vec
                    * - pin_element_vec
                    * - pw_element_vec * pin_element_vec - should use pin_element_vec register
                    * - 4 for horizontal_add()           - CPU should rewrite registers
                    * - pout_line[0]                     - should write straight to memory
                    *
                    * The horizontal add takes an inordinate amount of time, so we want to do it less frequently.  If we
                    * multiply the whole kernel and sum each row's dot product, we only need to do one horizontal add
                    * per matrix dot product.
                    *
                    * pin         - goes across the input            - points to the input line
                    * pin_line    - goes across the input line       - points to the input line block
                    * pin_element - goes across the input line block - points to the input line block row
                    * pout        - goes across the output           - points to the output line
                    * pout_line   - goes across the output line      - points to the output line "block"
                    * pw          - points to the window/kernel
                    * pw_element  - points to the window/kernel row
                    */
                   tiny_dnn::float_t *pout_line_vec = pout_vec;
                   const auto unroll_size = 2;
                   while (pout_line_vec <= pout_vec + 4 * (ow - unroll_size)) { // unrolled loop
                     // had to use assembly here because gcc 7.5 didn't tune to silvermont, despite the -mtune=silvermont flag
                     // here's the dependency graph, with cycle times.
                     // load takes 3 cycles, *= takes 5 (with a gap of 2 between *=), and add takes 3 cycles
                     /*
                      * out   in0   in1   in2
                      *  3     3     3     3
                      * sum   *k0 2 *k1 2 *k2
                      *        5     5     5
                      *       add   add   add
                      *        3     3     3
                      */
                     __m128 sum[2], pin_element_vec[2][3];
                     asm volatile ( "movups (%1), %0;" : "=x" (pin_element_vec[0][0]) : "r" (pin_line));
                     asm volatile ( "movups (%1, %2, 4), %0;" : "=x" (pin_element_vec[0][1]) : "r" (pin_line), "r" (iw * h_dilation_tiny_jnn));
                     asm volatile ( "movups (%1, %2, 8), %0;" : "=x" (pin_element_vec[0][2]) : "r" (pin_line), "r" (iw * h_dilation_tiny_jnn));
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element_vec[0][0]) : "x" (pw_element_vec[0]));
                     pin_line += elem_stride; // i could make this compile-time and embed it in the movups, but that seems less maintainable, and this way doesn't add any cpu cycles
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element_vec[0][1]) : "x" (pw_element_vec[1]));
                     asm volatile ( "movups (%1), %0;" : "=x" (pin_element_vec[1][0]) : "r" (pin_line));
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element_vec[0][2]) : "x" (pw_element_vec[2]));
                     asm volatile ( "movups (%1, %2, 4), %0;" : "=x" (pin_element_vec[1][1]) : "r" (pin_line), "r" (iw * h_dilation_tiny_jnn));
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element_vec[1][0]) : "x" (pw_element_vec[0]));
                     asm volatile ( "movups (%1, %2, 8), %0;" : "=x" (pin_element_vec[1][2]) : "r" (pin_line), "r" (iw * h_dilation_tiny_jnn));
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element_vec[1][1]) : "x" (pw_element_vec[1]));
                     asm volatile ( "movups (%1), %0;" : "=x" (sum[0]) : "r" (pout_line_vec));
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element_vec[1][2]) : "x" (pw_element_vec[2]));
                     asm volatile ( "movups 16(%1), %0;" : "=x" (sum[1]) : "r" (pout_line_vec));
                     asm volatile ( "addps %1, %0;" : "+x" (sum[0]) : "x" (pin_element_vec[0][0]));
                     asm volatile ( "addps %1, %0;" : "+x" (pin_element_vec[0][1]) : "x" (pin_element_vec[0][2]));
                     asm volatile ( "addps %1, %0;" : "+x" (sum[1]) : "x" (pin_element_vec[1][0]));
                     asm volatile ( "addps %1, %0;" : "+x" (pin_element_vec[1][1]) : "x" (pin_element_vec[1][2]));
                     asm volatile ( "addps %1, %0;" : "+x" (sum[0]) : "x" (pin_element_vec[0][1]));
                     pin_line += elem_stride; // this would be an otherwise unused cpu cycle, but gcc is putting it later.  TODO: figure out how to force gcc to put it here without screwing everything up
                     asm volatile ( "addps %1, %0;" : "+x" (sum[1]) : "x" (pin_element_vec[1][1]));
                     _mm_storeu_ps(pout_line_vec, sum[0]);
                     pout_line_vec += 4;
                     _mm_storeu_ps(pout_line_vec, sum[1]);
                     pout_line_vec += 4; // gcc is smart enough to merge this with the previous +=
                   }
                   for (size_t x = 0; x < ow % unroll_size; x++) { // did this before unrolling.  rarely run, so assembly wasn't necessary
                     __m128 sum, pin_element0_vec, pin_element1_vec, pin_element2_vec;
//                     pin_element0_vec = _mm_loadu_ps(pin_line);
                     asm volatile ( "movups (%1), %0;" : "=x" (pin_element0_vec) : "r" (pin_line));
//                     sum = _mm_loadu_ps(pout_line_vec);
                     asm volatile ( "movups (%1), %0;" : "=x" (sum) : "r" (pout_line_vec));
//                     pin_element1_vec = _mm_loadu_ps(pin_line + iw * h_dilation_tiny_jnn);
                     asm volatile ( "movups (%1, %2, 4), %0;" : "=x" (pin_element1_vec) : "r" (pin_line), "r" (iw * h_dilation_tiny_jnn));
//                     pin_element0_vec *= (__m128) pw_element_vec[0];
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element0_vec) : "x" (pw_element_vec[0]));
//                     pin_element2_vec = _mm_loadu_ps(pin_line + 2 * iw * h_dilation_tiny_jnn);
                     asm volatile ( "movups (%1, %2, 8), %0;" : "=x" (pin_element2_vec) : "r" (pin_line), "r" (iw * h_dilation_tiny_jnn));
//                     pin_element1_vec *= (__m128) pw_element_vec[1];
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element1_vec) : "x" (pw_element_vec[1]));
                     // unused cpu cycle
//                     pin_element2_vec *= (__m128) pw_element_vec[2];
                     asm volatile ( "mulps %1, %0;" : "+x" (pin_element2_vec) : "x" (pw_element_vec[2]));
//                     sum += pin_element0_vec;
                     asm volatile ( "addps %1, %0;" : "+x" (sum) : "x" (pin_element0_vec));
                     // unused cpu cycle
                     // unused cpu cycle
//                     sum += pin_element1_vec;
                     asm volatile ( "addps %1, %0;" : "+x" (sum) : "x" (pin_element1_vec));
                     // unused cpu cycle
                     // unused cpu cycle
//                     sum += pin_element2_vec;
                     asm volatile ( "addps %1, %0;" : "+x" (sum) : "x" (pin_element2_vec));
                     _mm_storeu_ps(pout_line_vec, sum);
                     pout_line_vec += 4;
                     pin_line += elem_stride;
                   }
                 } else if (window_width_tiny_jnn == 8 && window_height_tiny_jnn == 3 && w_dilation_tiny_jnn == 1) {
                   const __m128 pw_element0_0 = _mm_loadu_ps(pw);
                   const __m128 pw_element0_4 = _mm_loadu_ps(&pw[4]);
                   const __m128 pw_element1_0 = _mm_loadu_ps(&pw[8]);
                   const __m128 pw_element1_4 = _mm_loadu_ps(&pw[12]);
                   const __m128 pw_element2_0 = _mm_loadu_ps(&pw[16]);
                   const __m128 pw_element2_4 = _mm_loadu_ps(&pw[20]);
                   for (size_t x = 0; x < ow; x++) {
                     const tiny_dnn::float_t *pin_element = pin_line;
                     __m128 sum;
                     __m128 pin_element_vec;

                     pin_element_vec = _mm_loadu_ps(pin_element);
                     sum = pw_element0_0 * pin_element_vec;
                     pin_element_vec = _mm_loadu_ps(&pin_element[4]);
                     sum += pw_element0_4 * pin_element_vec;
                     pin_element += iw * h_dilation_tiny_jnn;

                     pin_element_vec = _mm_loadu_ps(pin_element);
                     sum += pw_element1_0 * pin_element_vec;
                     pin_element_vec = _mm_loadu_ps(&pin_element[4]);
                     sum += pw_element1_4 * pin_element_vec;
                     pin_element += iw * h_dilation_tiny_jnn;

                     pin_element_vec = _mm_loadu_ps(pin_element);
                     sum += pw_element2_0 * pin_element_vec;
                     pin_element_vec = _mm_loadu_ps(&pin_element[4]);
                     sum += pw_element2_4 * pin_element_vec;
                     pin_element += iw * h_dilation_tiny_jnn;

                     _mm_storeu_ps(pout_vec + 4 * x, _mm_loadu_ps(pout_vec + 4 * x) + sum);
                     pin_line += elem_stride;
                   }
                 } else {
                   for (size_t x = 0; x < ow; x++) {
                     const tiny_dnn::float_t *pin_element = pin_line;
                     const tiny_dnn::float_t *pw_element = pw;
                     tiny_dnn::float_t sum{0};
                     // should be optimized for small kernel(3x3,5x5)
                     for (size_t wy = 0; wy < window_height_tiny_jnn; wy++) {    // NOLINT
                       for (size_t wx = 0; wx < window_width_tiny_jnn; wx++) {  // NOLINT
                         sum += pw_element[wx] * pin_element[wx * w_dilation_tiny_jnn];
                       }
                       pw_element += window_width_tiny_jnn;
                       pin_element += iw * h_dilation_tiny_jnn;
                     }
                     pout[x] += sum;
                     pin_line += elem_stride;
                   }
                 }
                 pout += ow;
                 pout_vec += ow * 4;
                 pin += line_stride;
               }
             }
             tiny_dnn::float_t *pout      = pa;
             tiny_dnn::float_t *pout_vec  = pa_vec;
             for (size_t y = 0; y < oh; y++) {
               for (size_t x = 0; x < ow; x++) {
                 const auto sum_vec = _mm_loadu_ps(pout_vec + 4 * x);
                 if (window_width_tiny_jnn == 3 && window_height_tiny_jnn == 3 && w_dilation_tiny_jnn == 1) {
                   pout[x] = horizontal_add(sum_vec, 3);
                 } else if (window_width_tiny_jnn == 8 && window_height_tiny_jnn == 3 && w_dilation_tiny_jnn == 1) {
                   pout[x] = horizontal_add(sum_vec);
                 }
               }
               pout += ow;
               pout_vec += ow * 4;
             }
             if (params.has_bias) {
               vectorize::add(bias[o], out_area, pa);
             }
           }
         }
       },
       0u);
}

/******************************************************************/

template <typename tensor_t, typename vec_t>
void conv2d_op_internal(const tensor_t &prev_out,
                        const vec_t &W,
                        tensor_t &dW,
                        tensor_t &db,
                        tensor_t &curr_delta,
                        tensor_t &prev_delta,
                        const core::conv_params &params,
                        const bool parallelize) {
  typedef typename vec_t::value_type float_t;

  for_i(parallelize, prev_out.size(), [&](size_t sample) {
    // propagate delta to previous layer
    for (size_t inc = 0; inc < params.in.depth_; inc++) {
      for (size_t outc = 0; outc < params.out.depth_; outc++) {
        if (!params.tbl.is_connected(outc, inc)) continue;

        size_t idx        = 0;
        idx               = params.in.depth_ * outc + inc;
        idx               = params.weight.get_index(0, 0, idx);
        const float_t *pw = &W[idx];

        idx                       = params.out.get_index(0, 0, outc);
        const float_t *pdelta_src = &curr_delta[sample][idx];

        idx = params.in_padded.get_index(0, 0, inc);
        // float_t* pdelta_dst = &(*prev_delta)[sample][idx];
        float_t *pdelta_dst = &prev_delta[sample][idx];

        for (size_t y = 0; y < params.out.height_; y++) {
          for (size_t x = 0; x < params.out.width_; x++) {
            const float_t *ppw = pw;

            idx                       = y * params.out.width_ + x;
            const float_t ppdelta_src = pdelta_src[idx];

            float_t *ppdelta_dst =
              pdelta_dst + y * params.h_stride * params.in_padded.width_ +
              x * params.w_stride;

            for (size_t wy = 0; wy < params.weight.height_; wy++) {   // NOLINT
              for (size_t wx = 0; wx < params.weight.width_; wx++) {  // NOLINT
                idx = wy * params.in_padded.width_ + wx;
                ppdelta_dst[idx] += *ppw++ * ppdelta_src;
              }
            }
          }
        }
      }
    }

    // accumulate dw
    for (size_t inc = 0; inc < params.in.depth_; inc++) {
      for (size_t outc = 0; outc < params.out.depth_; outc++) {
        if (!params.tbl.is_connected(outc, inc)) continue;

        for (size_t wy = 0; wy < params.weight.height_; wy++) {
          for (size_t wx = 0; wx < params.weight.width_; wx++) {
            float_t dst{0};

            size_t idx           = 0;
            idx                  = params.in_padded.get_index(wx, wy, inc);
            const float_t *prevo = &prev_out[sample][idx];

            idx                  = params.out.get_index(0, 0, outc);
            const float_t *delta = &curr_delta[sample][idx];

            if (params.w_stride > 1) {
              for (size_t y = 0; y < params.out.height_; y++) {
                size_t prevo_idx =
                  y * params.in_padded.width_ * params.h_stride;
                size_t delta_idx = y * params.out.width_;

                for (size_t x = 0; x < params.out.width_; x++) {
                  dst += prevo[prevo_idx + x * params.w_stride] *
                         delta[delta_idx + x];
                }
              }
            } else {
              for (size_t y = 0; y < params.out.height_; y++) {
                dst += vectorize::dot(
                  prevo + y * params.in_padded.width_ * params.h_stride,
                  delta + y * params.out.width_, params.out.width_);
              }
            }

            idx = params.in.depth_ * outc + inc;
            dW[sample][params.weight.get_index(wx, wy, idx)] += dst;
          }
        }
      }
    }

    // accumulate db
    if (params.has_bias) {
      for (size_t outc = 0; outc < params.out.depth_; outc++) {
        size_t idx            = params.out.get_index(0, 0, outc);
        const float_t *delta  = &curr_delta[sample][idx];
        const float_t *deltaa = delta + params.out.width_ * params.out.height_;
        db[sample][outc] += std::accumulate(delta, deltaa, float_t{0});
      }
    }
  });
}

}  // namespace kernels
}  // namespace tiny_jnn
