#pragma once

#ifndef Q_MOC_RUN
#include <boost/thread.hpp>
#endif
#include <deque>
#include <vector>
#include "readers/reader.hpp"
#ifndef Q_MOC_RUN
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#endif

typedef int64_t msg_t;
typedef std::deque<msg_t> chat_message_queue;
class Connection;

/* A reader that connects with the nao and collects data which is
 * then stored in a naoData object.
 *
 */
class NetworkReader : public Reader {
   public:
      explicit NetworkReader(const QString &robotName, int robotPort, OffNaoMask_t mask);
      explicit NetworkReader(std::pair<std::pair<const QString &, int>,
                                       OffNaoMask_t> robotNameMask);
      explicit NetworkReader(std::pair<std::pair<const QString &, int>,
                                       OffNaoMask_t> robotNameMask,
                             const NaoData &naoData);
      ~NetworkReader();

      // main loop that runs when the thread starts
      virtual void run();

      /* Writes a message to the nao */
      void write(const msg_t &msg);
   private:
      OffNaoMask_t mask;
      boost::asio::io_service *ioservice;
      // WirelessClient *wirelessClient;

      void handle_connect(const boost::system::error_code& e,
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
      void handle_read(const boost::system::error_code& e);
      Connection *connection_;

      boost::thread *cthread;
      Frame received;

      boost::asio::ip::tcp::resolver *resolver;
      boost::asio::ip::tcp::resolver::query *query;

      bool isRecording;
      QString robotName;
      int robotPort;

      bool disconnect();
      bool connect();


      void do_write(msg_t msg);

      void handle_write(const boost::system::error_code &error);

      void do_close();

      chat_message_queue write_msgs_;
      public slots:
         virtual void stopMediaTrigger();
      virtual void recordMediaTrigger();
      /**
       * sends a command line string to the Nao
       */
      virtual void sendCommandLineString(std::vector<std::string> argv);
};
