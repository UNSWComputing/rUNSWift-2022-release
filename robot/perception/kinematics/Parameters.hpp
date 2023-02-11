#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

class Parameters {
   public:
      Parameters();

      float cameraPitchTop;
      float cameraYawTop;
      float cameraRollTop;

      float cameraYawBottom;
      float cameraPitchBottom;
      float cameraRollBottom;

      float bodyPitch;

      template<class Archive>
      void serialize(Archive &ar, const unsigned int file_version) {
         ar & cameraPitchTop;
         ar & cameraYawTop;
         ar & cameraRollTop;
         ar & cameraYawBottom;
         ar & cameraPitchBottom;
         ar & cameraRollBottom;
         ar & bodyPitch;
      }

};

#endif // PARAMETERS_HPP
