#include "serial_transport.h"
#include <ros/ros.h>

namespace abot {

SerialTransport::SerialTransport(std::string port, int32_t baudrate) : m_port(port), m_baudrate(baudrate), m_timeout_us(150 * 1000) {
  m_timeoutFlag = false;
}

SerialTransport::~SerialTransport() {
  if (m_fd) close(m_fd);
}

bool SerialTransport::init() {
  ROS_INFO("open %s %d", m_port.c_str(), m_baudrate);
  m_fd = ::open(m_port.c_str(), O_RDWR | O_NDELAY);
  if (m_fd < 0) {
    ROS_ERROR("open %s err", m_port.c_str());
    return false;
  }

  tcflush(m_fd, TCIOFLUSH);

  int n = fcntl(m_fd, F_GETFL, 0);
  fcntl(m_fd, F_SETFL, n & ~O_NDELAY);

  struct termios opt;
  tcgetattr(m_fd, &opt);

  if (m_baudrate == 921600) {
    cfsetispeed(&opt, B921600);
    cfsetospeed(&opt, B921600);
  } else if (m_baudrate == 1500000) {
    cfsetispeed(&opt, B1500000);
    cfsetospeed(&opt, B1500000);
  } else {  // if (m_baudrate == 115200)
    cfsetispeed(&opt, B115200);
    cfsetospeed(&opt, B115200);
  }

  opt.c_cflag &= ~CSIZE | CS8;
  opt.c_cflag |= (CLOCAL | CREAD);

  opt.c_cflag &= ~(PARENB | PARODD);
  opt.c_cflag &= ~CSTOPB;

  opt.c_cflag &= ~CRTSCTS;

  opt.c_iflag = IGNBRK;
  opt.c_iflag &= ~(IXON | IXOFF | IXANY);

  opt.c_lflag = 0;
  opt.c_oflag = 0;

  opt.c_cc[VMIN] = 0;
  opt.c_cc[VTIME] = 0;

  if ((tcsetattr(m_fd, TCSANOW, &opt)) != 0) {
    return false;
  }

  int mcs = 0;
  ioctl(m_fd, TIOCMGET, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(m_fd, TIOCMGET, &mcs);

  if (tcgetattr(m_fd, &opt) != 0) {
    ROS_ERROR("tcsetattr failed");
  }

  opt.c_cflag &= ~CRTSCTS;

  if (tcsetattr(m_fd, TCSANOW, &opt) != 0) {
    ROS_ERROR("tcsetattr failed");
  }

  return true;
}

Buffer SerialTransport::read() {
  Buffer data;

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(m_fd, &rfds);
  struct timeval tm;
  tm.tv_sec = 0;
  tm.tv_usec = m_timeout_us;

  int retval = select(m_fd + 1, &rfds, NULL, NULL, &tm);
  if (retval == -1 && errno == EINTR) {
    return data;
  }
  if (retval < 0) {
    return data;
  }

  if (!FD_ISSET(m_fd, &rfds)) {
    m_timeoutFlag = true;
    return data;
  }

  char buffer[256] = {0};
  int len = ::read(m_fd, buffer, sizeof(buffer));
  if (len > 0) {
#ifdef DEBUG_COMM
    printf("recv: ");
#endif
    for (int i = 0; i < len; i++) {
      data.push_back(buffer[i]);
#ifdef DEBUG_COMM
      printf("%02x ", (unsigned char)buffer[i]);
#endif
    }
#ifdef DEBUG_COMM
    printf("\r\n");
#endif
  } else {
    ROS_INFO("read err %d", len);
  }

  return data;
}

void SerialTransport::write(Buffer& data) {
#ifdef DEBUG_COMM
  printf("send: ");
  for (int i = 0; i < data.size(); i++) {
    printf("%02x ", (unsigned char)data[i]);
  }
  printf("\r\n");
#endif
  ::write(m_fd, data.data(), data.size());
}

void SerialTransport::set_timeout(int t) {
  m_timeout_us = t * 1000;
}

bool SerialTransport::is_timeout() {
  bool timeout = m_timeoutFlag;
  m_timeoutFlag = false;
  return timeout;
}
}  // namespace abot
