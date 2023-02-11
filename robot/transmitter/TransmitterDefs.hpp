#pragma once

#include <stdint.h>

/**
 * the type of what this is actually receiving.
 * historically used only by the off-nao transmitter.
 */
typedef uint64_t OffNaoMask_t;

enum {
   BLACKBOARD_MASK      = 0x0000000000000001ull,
   SALIENCY_MASK        = 0x0000000000000002ull,
   RAW_IMAGE_MASK       = 0x0000000000000004ull,
   PARTICLE_FILTER_MASK = 0x0000000000000008ull,
   ROBOT_FILTER_MASK    = 0x0000000000000010ull,
   INITIAL_MASK         = 0x0000000000000003ull,
   LANDMARKS_MASK       = 0x0000000000000020ull, // unused, can be recycled
   WHITEBOARD_MASK      = 0x0000000000000040ull,
   ALL_MASKS            = 0x000000000000007Full,
   USE_BATCHED_MASK     = 0x0000000000000080ull,
};
