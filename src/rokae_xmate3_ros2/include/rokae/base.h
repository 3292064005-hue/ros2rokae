#ifndef ROKAEAPI_BASE_H
#define ROKAEAPI_BASE_H

#include <chrono>
#include <functional>
#include <memory>
#include <system_error>

#if defined(_MSC_VER)
  #if defined(XCORESDK_DLL_BUILD)
    #define XCORE_API __declspec(dllexport)
  #elif defined(XCORESDK_DLL)
    #define XCORE_API __declspec(dllimport)
  #else
    #define XCORE_API
  #endif
#else
  #define XCORE_API __attribute__((visibility("default")))
#endif

namespace rokae {
template<class T> struct Base { };
}  // namespace rokae

typedef std::error_code error_code;

#endif  // ROKAEAPI_BASE_H
