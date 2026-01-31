// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef _FAST_DDS_GENERATED_PX4_MSGS_MSG_PX4ACTUATORCONTROLSPUBSUBTYPES_H_
#define _FAST_DDS_GENERATED_PX4_MSGS_MSG_PX4ACTUATORCONTROLSPUBSUBTYPES_H_

#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastrtps/utils/md5.h>

#include "PX4ActuatorControls.h"

#if !defined(GEN_API_VER) || (GEN_API_VER != 1)
#error Generated PX4ActuatorControls is not compatible with current installed Fast DDS. Please, regenerate it with fastddsgen.
#endif

namespace px4_msgs {
    namespace msg {
        #ifndef SWIG
        namespace detail {
            template<typename Tag, typename Tag::type M>
            struct PX4ActuatorControls_rob
            {
                friend constexpr typename Tag::type get(Tag) { return M; }
            };

            struct PX4ActuatorControls_f
            {
                typedef eprosima::fastcdr::Cdr& (eprosima::fastcdr::Cdr::*type)(PX4ActuatorControls&);
            };

            template struct PX4ActuatorControls_rob<PX4ActuatorControls_f, &eprosima::fastcdr::Cdr::deserialize<PX4ActuatorControls>>;

            template <typename T, typename Tag>
            inline size_t constexpr PX4ActuatorControls_offset_of() {
                return ((::size_t) &reinterpret_cast<char const volatile&>((((T*)0)->*get(Tag()))));
            }
        }
        #endif

        class PX4ActuatorControlsPubSubType : public eprosima::fastdds::dds::TopicDataType
        {
        public:
            typedef PX4ActuatorControls type;

            eProsima_user_DllExport PX4ActuatorControlsPubSubType();
            eProsima_user_DllExport virtual ~PX4ActuatorControlsPubSubType() override;
            eProsima_user_DllExport virtual bool serialize(void* data, eprosima::fastrtps::rtps::SerializedPayload_t* payload) override;
            eProsima_user_DllExport virtual bool deserialize(eprosima::fastrtps::rtps::SerializedPayload_t* payload, void* data) override;
            eProsima_user_DllExport virtual std::function<uint32_t()> getSerializedSizeProvider(void* data) override;
            eProsima_user_DllExport virtual bool getKey(void* data, eprosima::fastrtps::rtps::InstanceHandle_t* ihandle, bool force_md5 = false) override;
            eProsima_user_DllExport virtual void* createData() override;
            eProsima_user_DllExport virtual void deleteData(void* data) override;

        #ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
            eProsima_user_DllExport inline bool is_bounded() const override { return true; }
        #endif

        #ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
            eProsima_user_DllExport inline bool is_plain() const override { return false; }
        #endif

        #ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
            eProsima_user_DllExport inline bool construct_sample(void* memory) const override {
                (void)memory;
                return false;
            }
        #endif

        private:
            eprosima::fastrtps::rtps::MD5 m_md5;
            unsigned char* m_keyBuffer;
        };
    }
}

#endif // _FAST_DDS_GENERATED_PX4_MSGS_MSG_PX4ACTUATORCONTROLSPUBSUBTYPES_H_
