#pragma once

// Define IGCCLIB_API for any platform
#if defined _WIN32 || defined __CYGWIN__
  #ifdef WIN_EXPORT
    // Exporting...
    #ifdef __GNUC__
      #define IGCCLIB_API __attribute__ ((dllexport))
    #else
      #define IGCCLIB_API __declspec(dllexport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #else
    #ifdef __GNUC__
      #define IGCCLIB_API __attribute__ ((dllimport))
    #else
      #define IGCCLIB_API __declspec(dllimport) // Note: actually gcc seems to also supports this syntax.
    #endif
  #endif
  #define NOT_IGCCLIB_API
#else
  #if __GNUC__ >= 4
    #define IGCCLIB_API __attribute__ ((visibility ("default")))
    #define NOT_IGCCLIB_API  __attribute__ ((visibility ("hidden")))
  #else
    #define IGCCLIB_API
    #define NOT_IGCCLIB_API
  #endif
#endif