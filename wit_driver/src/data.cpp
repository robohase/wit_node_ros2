/**
 * @file /xbot_arm_driver/src/driver/core_sensors.cpp
 *
 * @brief Implementation of the core sensor packet data.
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/xbot_core/hydro-devel/xbot_arm_driver/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <time.h>
#include "wit_driver/data.hpp"
#include "wit_driver/packet_handler/payload_headers.hpp"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace wit {

/*****************************************************************************
** Implementation
*****************************************************************************/

bool Data::serialise(ecl::PushAndPop<unsigned char> &) { return true; }

bool Data::desTime(ecl::PushAndPop<unsigned char> &byteStream) {
  uint8_t tmpL = 0;
  // uint8_t tmpH;
  uint16_t tmp = 0;
  buildVariable(tmpL, byteStream); // 20YY
                                   //  std::cout << int(tmpL) << std::endl;
  buildVariable(tmpL, byteStream); // MM
  buildVariable(tmpL, byteStream); // DD
  imugps_.timestamp = static_cast<double>(tmpL) * (24.0 * 60.0 * 60.0);
  buildVariable(tmpL, byteStream); // HH
  imugps_.timestamp += static_cast<double>(tmpL) * (60.0 * 60.0);
  buildVariable(tmpL, byteStream); // MM
  imugps_.timestamp += static_cast<double>(tmpL) * 60.0;
  buildVariable(tmpL, byteStream); // SS
  imugps_.timestamp += static_cast<double>(tmpL);
  buildVariable(tmp, byteStream); // MSL,MSH
  imugps_.timestamp += tmp * 0.001;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.timestamp << std::endl;
    return true;
  } else {
    return false;
  }
}

bool Data::desAcce(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp = 0;
  buildVariable(tmp, byteStream); // accx
  imugps_.a[0] = static_cast<double>(tmp) * (16.0 * 9.8 / 32768.0);
  buildVariable(tmp, byteStream); // accy
  imugps_.a[1] = static_cast<double>(tmp) * (16.0 * 9.8 / 32768.0);
  buildVariable(tmp, byteStream); // accz
  imugps_.a[2] = static_cast<double>(tmp) * (16.0 * 9.8 / 32768.0);
  buildVariable(tmp, byteStream);
  imugps_.temperature = static_cast<double>(tmp) * 0.01;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.a[2] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desGyro(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp = 0;
  buildVariable(tmp, byteStream); // wx
  imugps_.w[0] = static_cast<double>(tmp) * (2000.0 * 3.1415926 / (32768.0 * 180.0));
  buildVariable(tmp, byteStream); // wy
  imugps_.w[1] = static_cast<double>(tmp) * (2000.0 * 3.1415926 / (32768.0 * 180.0));
  buildVariable(tmp, byteStream); // wz
  imugps_.w[2] = static_cast<double>(tmp) * (2000.0 * 3.1415926 / (32768.0 * 180.0));
  buildVariable(tmp, byteStream);
  imugps_.temperature = static_cast<double>(tmp) * 0.01;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.w[0] << std::endl;
    return true;
  } else {
    return false;
  }
}

// Given the measurement as a float (incorrectly), convert it to decimal degrees.
static float fixBits(const float measurement) {
  const int int_measurement = reinterpret_cast<const int &>(measurement);
  float degrees = int_measurement * 1e-7;
  float minutes_and_seconds = (int_measurement % (int)1e7) * 1e-7f;
  return degrees + minutes_and_seconds * (100.0f / 60.0f);
}

bool Data::desLola(ecl::PushAndPop<unsigned char> &byteStream) {
  buildVariable(imugps_.longtitude, byteStream);
  buildVariable(imugps_.latitude, byteStream);

  imugps_.latitude = fixBits(imugps_.latitude);
  imugps_.longtitude = fixBits(imugps_.longtitude);
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.pressure << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desMag(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp = 0;
  buildVariable(tmp, byteStream); // magx
  imugps_.mag[0] = static_cast<double>(tmp);
  buildVariable(tmp, byteStream); // magy
  imugps_.mag[1] = static_cast<double>(tmp);
  buildVariable(tmp, byteStream); // magz
  imugps_.mag[2] = static_cast<double>(tmp);
  buildVariable(tmp, byteStream);
  imugps_.temperature = static_cast<double>(tmp) * 0.01;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.mag[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desPalt(ecl::PushAndPop<unsigned char> &byteStream) {
  buildVariable(imugps_.pressure, byteStream);
  buildVariable(imugps_.altitude, byteStream);
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.pressure << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desQuat(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp = 0;
  buildVariable(tmp, byteStream);
  imugps_.q[0] = static_cast<double>(tmp) * (1.0 / 32768.0);
  buildVariable(tmp, byteStream);
  imugps_.q[1] = static_cast<double>(tmp) * (1.0 / 32768.0);
  buildVariable(tmp, byteStream);
  imugps_.q[2] = static_cast<double>(tmp) * (1.0 / 32768.0);
  buildVariable(tmp, byteStream);
  imugps_.q[3] = static_cast<double>(tmp) * (1.0 / 32768.0);
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.q[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desSate(ecl::PushAndPop<unsigned char> &byteStream) {
  buildVariable(imugps_.satelites, byteStream);
  uint16_t tmp = 0;
  buildVariable(tmp, byteStream);
  imugps_.gpsa[0] = static_cast<double>(tmp) * 0.01;
  buildVariable(tmp, byteStream);
  imugps_.gpsa[1] = static_cast<double>(tmp) * 0.01;
  buildVariable(tmp, byteStream);
  imugps_.gpsa[2] = static_cast<double>(tmp) * 0.01;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.gpsa[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desState(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp = 0;
  buildVariable(tmp, byteStream); // D0
  imugps_.d[0] = static_cast<uint16_t>(tmp);
  buildVariable(tmp, byteStream); // D1
  imugps_.d[1] = static_cast<uint16_t>(tmp);
  buildVariable(tmp, byteStream); // D2
  imugps_.d[2] = static_cast<uint16_t>(tmp);
  buildVariable(tmp, byteStream); // D3
  imugps_.d[3] = static_cast<uint16_t>(tmp);
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.d[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desRpy(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp = 0;
  buildVariable(tmp, byteStream); // roll
  imugps_.rpy[0] = static_cast<double>(tmp) * (M_PI / 32768.0);
  buildVariable(tmp, byteStream); // pitch
  imugps_.rpy[1] = static_cast<double>(tmp) * (M_PI / 32768.0);
  buildVariable(tmp, byteStream); // yaw
  imugps_.rpy[2] = static_cast<double>(tmp) * (M_PI / 32768.0);
  buildVariable(tmp, byteStream);
  imugps_.temperature = static_cast<double>(tmp) * 0.01;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.rpy[0] << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::desGpsv(ecl::PushAndPop<unsigned char> &byteStream) {
  int16_t tmp = 0;
  buildVariable(tmp, byteStream); // gpsh
  imugps_.gpsh = static_cast<double>(tmp) * 0.1;
  buildVariable(tmp, byteStream); // gpsy
  imugps_.gpsy = static_cast<double>(tmp) * 0.1;
  buildVariable(imugps_.gpsv, byteStream); // accz
  imugps_.gpsv *= 0.001;
  if (byteStream.size() == 0) {
    //    std::cout << imugps_.gpsv << std::endl;
    return true;
  } else {
    return false;
  }
}
bool Data::deserialise(ecl::PushAndPop<unsigned char> &byteStream) {
  uint8_t flag = 0;
  buildVariable(flag, byteStream);
  switch (flag) {
    case Flags::TIME:
      desTime(byteStream);
      break;
    case Flags::ACCE:
      desAcce(byteStream);
      break;
    case Flags::GYRO:
      desGyro(byteStream);
      break;
    case Flags::RPY:
      desRpy(byteStream);
      break;
    case Flags::MAG:
      desMag(byteStream);
      break;
    case Flags::STATE:
      desState(byteStream);
      break;
    case Flags::PALT:
      desPalt(byteStream);
      break;
    case Flags::LOLA:
      desLola(byteStream);
      break;
    case Flags::GPSV:
      desGpsv(byteStream);
      break;
    case Flags::QUAT:
      desQuat(byteStream);
      break;
    case Flags::SATE:
      desSate(byteStream);
      break;
    default:
      break;
  }

  return true;
}

void Data::build_special_variable(float &variable, ecl::PushAndPop<unsigned char> &byteStream) {
  if (byteStream.size() < 2) return;

  unsigned char a = 0, b = 0;
  buildVariable(a, byteStream);
  buildVariable(b, byteStream);
  variable = static_cast<float>((unsigned int)(a & 0x0f)) * 0.01f;

  variable += static_cast<float>((unsigned int)(a >> 4)) * 0.1f;

  variable += static_cast<float>((unsigned int)(b & 0x0f));
  variable += static_cast<float>((unsigned int)(b >> 4)) * 10.0f;
}

} // namespace wit
