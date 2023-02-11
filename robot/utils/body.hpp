/**
 * Body.hpp
 * Description: Contains Joint Codes, Limits, Motor Reductions and Limb
 *              lengths and weights
 */

#ifndef BODY_HPP
#define BODY_HPP

#include "bodyV5.hpp"
#include "bodyV6.hpp"

#ifdef CTC_2_1
namespace Joints = V5Joints;
namespace Limbs = V5Limbs;
namespace LEDs = V5LEDs;
namespace Sensors = V5Sensors;
#else
namespace Joints = V6Joints;
namespace Limbs = V6Limbs;
namespace LEDs = V6LEDs;
namespace Sensors = V6Sensors;
#endif

#endif // BODY_HPP
