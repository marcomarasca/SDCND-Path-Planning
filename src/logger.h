#ifndef PP_LOGGER_H
#define PP_LOGGER_H

#include <iostream>
#include <string>

namespace PathPlanning {

enum LogLevel { DEBUG, INFO, WARN, ERROR };

struct LogConfig {
  bool enabled = true;
  LogLevel level = INFO;
};

extern LogConfig LOG_CONFIG;

class LOG {
 public:
  LOG() {}
  LOG(LogLevel level) {
    this->level = level;
    operator<<(this->Prefix(level));
  }
  ~LOG() {
    if (this->open) {
      std::cout << std::endl;
    }
    this->open = false;
  }
  template <class T>
  LOG &operator<<(const T &msg) {
    if (LOG_CONFIG.enabled && this->level >= LOG_CONFIG.level) {
      std::cout << msg;
      open = true;
    }
    return *this;
  }

 private:
  bool open = false;
  LogLevel level = DEBUG;
  inline std::string Prefix(LogLevel type) {
    std::string prefix;
    switch (type) {
      case DEBUG:
        prefix = "[DEBUG]: ";
        break;
      case INFO:
        prefix = "[INFO]:  ";
        break;
      case WARN:
        prefix = "[WARN]:  ";
        break;
      case ERROR:
        prefix = "[ERROR]: ";
        break;
    }
    return prefix;
  }
};

}  // namespace PathPlanning

#endif