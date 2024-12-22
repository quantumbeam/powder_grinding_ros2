// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GRINDING_SCENE_DESCRIPTION__VISIBILITY_H_
#define GRINDING_SCENE_DESCRIPTION__VISIBILITY_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

  #ifdef __GNUC__
    #define GRINDING_SCENE_DESCRIPTION_EXPORT __attribute__ ((dllexport))
    #define GRINDING_SCENE_DESCRIPTION_IMPORT __attribute__ ((dllimport))
  #else
    #define GRINDING_SCENE_DESCRIPTION_EXPORT __declspec(dllexport)
    #define GRINDING_SCENE_DESCRIPTION_IMPORT __declspec(dllimport)
  #endif

  #ifdef GRINDING_SCENE_DESCRIPTION_DLL
    #define GRINDING_SCENE_DESCRIPTION_PUBLIC GRINDING_SCENE_DESCRIPTION_EXPORT
  #else
    #define GRINDING_SCENE_DESCRIPTION_PUBLIC GRINDING_SCENE_DESCRIPTION_IMPORT
  #endif

  #define GRINDING_SCENE_DESCRIPTION_PUBLIC_TYPE GRINDING_SCENE_DESCRIPTION_PUBLIC

  #define GRINDING_SCENE_DESCRIPTION_LOCAL

#else

  #define GRINDING_SCENE_DESCRIPTION_EXPORT __attribute__ ((visibility("default")))
  #define GRINDING_SCENE_DESCRIPTION_IMPORT

  #if __GNUC__ >= 4
    #define GRINDING_SCENE_DESCRIPTION_PUBLIC __attribute__ ((visibility("default")))
    #define GRINDING_SCENE_DESCRIPTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GRINDING_SCENE_DESCRIPTION_PUBLIC
    #define GRINDING_SCENE_DESCRIPTION_LOCAL
  #endif

  #define GRINDING_SCENE_DESCRIPTION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // GRINDING_SCENE_DESCRIPTION__VISIBILITY_H_
