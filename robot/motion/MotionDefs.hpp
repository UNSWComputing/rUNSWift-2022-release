#ifndef MOTION_DEFS_HPP
#define MOTION_DEFS_HPP

#ifdef CTC_2_1
#define MOTION_DT 0.010  // 100 Hz motion thread
#else
#define MOTION_DT 0.0122   // 81.9672131147541 Hz motion thread
#endif

#endif //  MOTION_DEFS_HPP