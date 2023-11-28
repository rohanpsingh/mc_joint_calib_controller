#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define JointCalibController_DLLIMPORT __declspec(dllimport)
#  define JointCalibController_DLLEXPORT __declspec(dllexport)
#  define JointCalibController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define JointCalibController_DLLIMPORT __attribute__((visibility("default")))
#    define JointCalibController_DLLEXPORT __attribute__((visibility("default")))
#    define JointCalibController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define JointCalibController_DLLIMPORT
#    define JointCalibController_DLLEXPORT
#    define JointCalibController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef JointCalibController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define JointCalibController_DLLAPI
#  define JointCalibController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef JointCalibController_EXPORTS
#    define JointCalibController_DLLAPI JointCalibController_DLLEXPORT
#  else
#    define JointCalibController_DLLAPI JointCalibController_DLLIMPORT
#  endif // JointCalibController_EXPORTS
#  define JointCalibController_LOCAL JointCalibController_DLLLOCAL
#endif // JointCalibController_STATIC
