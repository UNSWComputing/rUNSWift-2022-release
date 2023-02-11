#pragma once

#ifndef Q_MOC_RUN
#include <boost/serialization/version.hpp>
#endif

struct CameraSettings {
    explicit CameraSettings();
    unsigned int hflip;
    unsigned int vflip;
    unsigned int brightness;
    unsigned int contrast;
    unsigned int saturation;
    unsigned int hue;
    unsigned int sharpness;
    unsigned int backlightCompensation;
    unsigned int exposure;
    unsigned int gain;
    unsigned int whiteBalance;
    unsigned int exposureAuto;
    unsigned int autoWhiteBalance;
    unsigned int autoFocus;
    unsigned int focusAbsolute;
    unsigned int exposureAlgorithm;
    unsigned int aeTargetAvgLuma;
    unsigned int aeTargetAvgLumaDark;
    unsigned int aeTargetGain;
    unsigned int aeMinVirtGain;
    unsigned int aeMaxVirtGain;
    unsigned int aeMinVirtAGain;
    unsigned int aeMaxVirtAGain;
    unsigned int aeTargetExposure;
    bool         aeUseWeightTable;
    float        aeWeightTableX1;
    float        aeWeightTableX2;
    float        aeWeightTableY1;
    float        aeWeightTableY2;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
      ar & hflip;
      ar & vflip;
      ar & brightness;
      ar & contrast;
      ar & saturation;
      ar & hue;
      ar & sharpness;
      ar & backlightCompensation;
      ar & exposure;
      ar & gain;

      if (file_version < 1)
      {
        int tmp;
        ar & tmp;
        whiteBalance = (unsigned int) tmp;
      }

      if (file_version >= 1){
        ar & whiteBalance;
        ar & exposureAuto;
        ar & autoWhiteBalance;
        ar & exposureAlgorithm;
        ar & aeTargetAvgLuma;
        ar & aeTargetAvgLumaDark;
        ar & aeTargetGain;
        ar & aeMinVirtGain;
        ar & aeMaxVirtGain;
        ar & aeMinVirtAGain;
        ar & aeMaxVirtAGain;
        ar & autoFocus;
      }

      if (file_version >= 2)
      {
        ar & aeTargetExposure;
      }

      if (file_version >= 3)
      {
        ar & aeUseWeightTable;
      }
    }
};

BOOST_CLASS_VERSION(CameraSettings, 3);
