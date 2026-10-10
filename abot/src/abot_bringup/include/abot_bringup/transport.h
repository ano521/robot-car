#ifndef ABOT_BRINGUP_TRANSPORT_H_
#define ABOT_BRINGUP_TRANSPORT_H_

#include <iostream>
#include <inttypes.h>
#include <vector>
#include <deque>
#include <queue>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>

namespace abot {

typedef std::vector<uint8_t> Buffer;

class Transport {
 public:
  virtual ~Transport() {
  }
  virtual bool init() = 0;
  virtual void set_timeout(int t) = 0;
  virtual bool is_timeout() = 0;
  virtual Buffer read() = 0;

  virtual void write(Buffer& data) = 0;
};

}  // namespace abot

#endif
