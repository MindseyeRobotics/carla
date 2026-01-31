// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef _FAST_DDS_GENERATED_PX4_MSGS_MSG_PX4ACTUATORCONTROLS_H_
#define _FAST_DDS_GENERATED_PX4_MSGS_MSG_PX4ACTUATORCONTROLS_H_

#include "Header.h"

#include <fastrtps/utils/fixed_size_string.hpp>

#include <stdint.h>
#include <array>
#include <string>
#include <vector>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(PX4ActuatorControls_SOURCE)
#define PX4ActuatorControls_DllAPI __declspec( dllexport )
#else
#define PX4ActuatorControls_DllAPI __declspec( dllimport )
#endif
#else
#define PX4ActuatorControls_DllAPI
#endif
#else
#define PX4ActuatorControls_DllAPI
#endif

namespace eprosima {
namespace fastcdr {
class Cdr;
}
}

namespace px4_msgs {
    namespace msg {
        class PX4ActuatorControls
        {
        public:
            eProsima_user_DllExport PX4ActuatorControls();
            eProsima_user_DllExport ~PX4ActuatorControls();
            eProsima_user_DllExport PX4ActuatorControls(const PX4ActuatorControls& x);
            eProsima_user_DllExport PX4ActuatorControls(PX4ActuatorControls&& x) noexcept;
            eProsima_user_DllExport PX4ActuatorControls& operator=(const PX4ActuatorControls& x);
            eProsima_user_DllExport PX4ActuatorControls& operator=(PX4ActuatorControls&& x) noexcept;
            eProsima_user_DllExport bool operator==(const PX4ActuatorControls& x) const;
            eProsima_user_DllExport bool operator!=(const PX4ActuatorControls& x) const;

            eProsima_user_DllExport void timestamp(uint64_t _timestamp);
            eProsima_user_DllExport uint64_t timestamp() const;
            eProsima_user_DllExport uint64_t& timestamp();

            eProsima_user_DllExport void control(const std::array<float, 8>& _control);
            eProsima_user_DllExport void control(std::array<float, 8>&& _control);
            eProsima_user_DllExport const std::array<float, 8>& control() const;
            eProsima_user_DllExport std::array<float, 8>& control();

            eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static size_t getCdrSerializedSize(const PX4ActuatorControls& data, size_t current_alignment = 0);
            eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr& cdr) const;
            eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr& cdr);
            eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);
            eProsima_user_DllExport static bool isKeyDefined();
            eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr& cdr) const;

        private:
            uint64_t m_timestamp;
            std::array<float, 8> m_control;
        };
    }
}

#endif // _FAST_DDS_GENERATED_PX4_MSGS_MSG_PX4ACTUATORCONTROLS_H_
