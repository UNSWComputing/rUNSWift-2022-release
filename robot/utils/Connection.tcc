#include <iostream>
#include "utils/Logger.hpp"
#include "utils/snappy/snappy.h"

template <typename T, typename Handler>
void Connection::async_write(const T& t, Handler handler) {
   // Serialise the data first so we know how large it is.
   serialize(t, outbound_data_);
   llog(DEBUG1) << outbound_data_.size();

   // Compress it
   size_t compressedSize;
   char* compressedBuffer = new char[snappy::MaxCompressedLength(outbound_data_.size())];
   snappy::RawCompress(
      outbound_data_.c_str(), outbound_data_.size(),
      compressedBuffer, &compressedSize
   );

   // Format the header.
   std::ostringstream header_stream;
   header_stream << std::setw(kHeaderLength) << std::hex << compressedSize <<
   std::setw(kHeaderLength) << outbound_data_.size();
   if (!header_stream || header_stream.str().size() != kHeaderLength * 2) {
      // Something went wrong, inform the caller.
      // static so it still exists when it's read
      static const ErrorCategory &category = ErrorCategory("async_write");
      boost::system::error_code error(boost::asio::error::invalid_argument, category);
      socket_.get_io_service().post(boost::bind(handler, error));
      return;
   }
   outbound_header_ = header_stream.str();

   // Write the serialized data to the socket. We use "gather-write" to send
   // both the header and the data in a single write operation.
   std::vector<boost::asio::const_buffer> buffers;
   buffers.push_back(boost::asio::buffer(outbound_header_));
   if (compressedSize)
      buffers.push_back(boost::asio::buffer(compressedBuffer, compressedSize));
   else
      buffers.push_back(boost::asio::buffer(outbound_data_));
   llog(DEBUG1) << outbound_header_ << std::endl;
   boost::asio::async_write(socket_, buffers, handler);
   // TODO (Peter): We don't know when the async_write will complete and we can
   // TODO: .... safely clean up the 'new'-ly allocated char* compressedBuffer
   // delete [] compressedBuffer;
}

template <typename T, typename Handler>
void Connection::async_read(T& t, Handler handler) {
   // Issue a read operation to read exactly the number of bytes in a header.
   void (Connection::*f)(
      const boost::system::error_code &,
      T &, boost::tuple<Handler>)
      = &Connection::handle_read_header<T, Handler>;
   boost::asio::async_read(socket_, boost::asio::buffer(inbound_header_),
                           boost::bind(f,
                                       this, boost::asio::placeholders::error, boost::ref(t),
                                       boost::make_tuple(handler)));
}

template <typename T>
void Connection::serialize(const T& t, std::string& data) {
   std::ostringstream archive_stream;
   serialise(t, archive_stream);
   data = archive_stream.str();
}

template <typename T>
boost::system::error_code Connection::sync_write(const T& t) {
    // If we've switched back to a normal sync, expire the buffer
    if (curr_buffer_size_ > 0) {
        clear_buffer();
    }
    serialize(t, outbound_data_);
    return sync_send(outbound_data_);
}

template <typename T>
boost::system::error_code Connection::batch_write(const T& t) {
    if (curr_buffer_size_ == OUTBOUND_BUFFER_SIZE) {
        boost::system::error_code retval;
        for (int i = 0; i < OUTBOUND_BUFFER_SIZE; i++) {
            retval = sync_send(buffered_outbound_data_[i]);
            buffered_outbound_data_[i].clear();

            if (retval != boost::system::errc::success) {
                // if we ever have a non-successfuly result, clear remaining buffers and exit loop
                clear_buffer(i);
            }
        }
        curr_buffer_size_ = 0;
        return retval;
    } else {
        serialize(t, buffered_outbound_data_[curr_buffer_size_]);
        curr_buffer_size_++;
        return boost::system::errc::make_error_code(boost::system::errc::success);
    }
}


template <typename T>
boost::system::error_code Connection::sync_read(T& t) {
   // Issue a read operation to read exactly the number of bytes in a header.
   // TODO(jayen): check for errors
   boost::asio::read(socket_, boost::asio::buffer(inbound_header_));
   // Determine the length of the serialized data.
   std::istringstream is(std::string(inbound_header_, kHeaderLength * 2));
   std::size_t inbound_compressed_size = 0;
   std::size_t inbound_data_size = 0;
   // static so it still exists when it's read
   const ErrorCategory &category = ErrorCategory("sync_read");
   if (!(is >> std::hex >> inbound_compressed_size >> inbound_data_size)) {
      // Header doesn't seem to be valid. Inform the caller.
      boost::system::error_code error(boost::asio::error::invalid_argument, category);
      return error;
   }
   llog(DEBUG1) << "compressed:\t" << inbound_compressed_size << "\tuncompressed:\t" << inbound_data_size << std::endl;

   // Start a synchronous call to receive the data.
   inbound_data_.resize(inbound_data_size);
   if (inbound_compressed_size == 0) {
      // TODO(jayen): check for errors
      boost::asio::read(socket_, boost::asio::buffer(inbound_data_));
   } else {
      inbound_compressed_data_.resize(inbound_compressed_size);
      // TODO(jayen): check for errors
      boost::asio::read(socket_, boost::asio::buffer(inbound_compressed_data_));

      // Decompress it
      // returns false if the message is corrupted and could not be decrypted
      bool ok = snappy::RawUncompress(
         &inbound_compressed_data_[0],
         inbound_compressed_data_.size(),
         &inbound_data_[0]
      );
      if (!ok) {
         llog(ERROR) << "failed to snappy decompress" << std::endl;
      }
   }

   // Extract the data structure from the data just received.
   try {
      std::string archive_data(&inbound_data_[0], inbound_data_.size());
      llog(DEBUG1) << std::string(inbound_header_, kHeaderLength * 2) <<
      std::endl;
      std::istringstream archive_stream(archive_data);
      deserialise(t, archive_stream);
   } catch(std::exception & e) {
      // Unable to decode data.
      boost::system::error_code error(boost::asio::error::invalid_argument, category);
      return error;
   }
   return boost::system::errc::make_error_code(boost::system::errc::success);
}

template <typename T, typename Handler>
void Connection::handle_read_header(const boost::system::error_code& e,
                                    T& t, boost::tuple<Handler> handler) {
   if (e) {
      boost::get<0>(handler) (e);
   } else {
      // Determine the length of the serialized data.
      std::istringstream is(std::string(inbound_header_, kHeaderLength * 2));
      std::size_t inbound_compressed_size = 0;
      std::size_t inbound_data_size = 0;
      if (!(is >> std::hex >> inbound_compressed_size >> inbound_data_size)) {
         // Header doesn't seem to be valid. Inform the caller.
         // static so it still exists when it's read
         const ErrorCategory &category = ErrorCategory("handle_read_header");
         boost::system::error_code error(boost::asio::error::invalid_argument, category);
         boost::get<0>(handler) (error);
         return;
      }
      llog(DEBUG1) << inbound_compressed_size << inbound_data_size << std::endl;

      // Start an asynchronous call to receive the data.
      inbound_compressed_data_.resize(inbound_compressed_size);
      inbound_data_.resize(inbound_data_size);
      void (Connection::*f)(const boost::system::error_code &, T &,
                            boost::tuple<Handler>) =
         &Connection::handle_read_data<T, Handler>;
      if (inbound_compressed_size)
         boost::asio::async_read(socket_,
                                 boost::asio::buffer(inbound_compressed_data_),
                                 boost::bind(f, this,
                                             boost::asio::placeholders::error,
                                             boost::ref(t), handler));
      else
         boost::asio::async_read(socket_, boost::asio::buffer(inbound_data_),
                                 boost::bind(f, this,
                                             boost::asio::placeholders::error,
                                             boost::ref(t), handler));
   }
}

template <typename T, typename Handler>
void Connection::handle_read_data(const boost::system::error_code& e,
                                  T& t, boost::tuple<Handler> handler) {
   if (e) {
      boost::get<0>(handler) (e);
   } else {
      if (!inbound_compressed_data_.empty()) {
         // Decompress it
         // returns false if the message is corrupted and could not be decrypted
         bool ok = snappy::RawUncompress(
            &inbound_compressed_data_[0],
            inbound_compressed_data_.size(),
            &inbound_data_[0]
         );
         if (!ok) {
            llog(ERROR) << "failed to snappy decompress" << std::endl;
         }
      }

      // Extract the data structure from the data just received.
      try {
         std::string archive_data(&inbound_data_[0], inbound_data_.size());
         llog(DEBUG1) << std::string(inbound_header_, kHeaderLength * 2) <<
         std::endl;
         std::istringstream archive_stream(archive_data);
         deserialise(t, archive_stream);
      } catch(std::exception & e) {
         // Unable to decode data.
         // static so it still exists when it's read
         static const ErrorCategory &category = ErrorCategory("handle_read_data");
         boost::system::error_code error(boost::asio::error::invalid_argument, category);
         boost::get<0>(handler) (error);
         return;
      }

      // Inform caller that data has been received ok.
      boost::get<0>(handler) (e);
   }
}
