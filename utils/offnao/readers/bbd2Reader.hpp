#pragma once

#include "reader.hpp"

#ifndef Q_MOC_RUN
#include <boost/archive/binary_iarchive.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#endif
#include <fstream>
#include <QString>

/*
 *  Simple reader that reads in recorded dumps from file.
 */
class BBD2Reader : public Reader {
   Q_OBJECT
   public:
      /**
       * sets up necessary file descriptors for reading
       *
       * @param fileName the name of the file to read
       */
      explicit BBD2Reader(const QString &fileName);

      /**
       * sets up necessary file descriptors for reading
       *
       * @param fileName the name of the file to read
       * @param naoData the old naoData to append to
       */
      explicit BBD2Reader(const QString &fileName, const NaoData &naoData);

      virtual void run();

      /**
       * closes appropriate file descriptors and frees memory
       */
      virtual ~BBD2Reader();
   private:
      /**
       * the file stream to read from
       */
      std::ifstream ifs;
};
