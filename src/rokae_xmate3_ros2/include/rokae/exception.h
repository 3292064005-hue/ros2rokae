#ifndef ROKAE_EXCEPTION_H
#define ROKAE_EXCEPTION_H

#include "rokae/base.h"

#include <stdexcept>
#include <string>
#include <system_error>

namespace rokae {

class XCORE_API Exception : public std::runtime_error {
 public:
  explicit Exception(const std::string &message, std::error_code ec = {})
      : std::runtime_error(message), error_(std::move(ec)) {}

  [[nodiscard]] const std::error_code &error() const noexcept { return error_; }

 private:
  std::error_code error_;
};

class XCORE_API ExecutionException : public Exception {
 public:
  using Exception::Exception;
};

class XCORE_API RealtimeControlException : public Exception {
 public:
  using Exception::Exception;
};

class XCORE_API RealtimeStateException : public Exception {
 public:
  using Exception::Exception;
};

class XCORE_API RealtimeMotionException : public Exception {
 public:
  using Exception::Exception;
};

class XCORE_API RealtimeParameterException : public Exception {
 public:
  using Exception::Exception;
};

inline std::string compose_exception_message(const std::string &context, const std::error_code &ec) {
  if (!ec) {
    return context;
  }
  if (context.empty()) {
    return ec.message();
  }
  return context + ": " + ec.message();
}

template <typename ExceptionT>
inline void throw_if_error(const std::error_code &ec, const std::string &context) {
  if (ec) {
    throw ExceptionT(compose_exception_message(context, ec), ec);
  }
}

}  // namespace rokae

#endif
