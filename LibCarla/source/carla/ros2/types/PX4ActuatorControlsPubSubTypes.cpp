// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "PX4ActuatorControlsPubSubTypes.h"
#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;
using InstanceHandle_t = eprosima::fastrtps::rtps::InstanceHandle_t;

namespace px4_msgs {
namespace msg {

PX4ActuatorControlsPubSubType::PX4ActuatorControlsPubSubType() {
    setName("px4_msgs::msg::dds_::PX4ActuatorControls_");
    auto type_size = PX4ActuatorControls::getMaxCdrSerializedSize();
    type_size += eprosima::fastcdr::Cdr::alignment(type_size, 4);
    m_typeSize = static_cast<uint32_t>(type_size) + 4;
    m_isGetKeyDefined = PX4ActuatorControls::isKeyDefined();
    size_t keyLength = PX4ActuatorControls::getKeyMaxCdrSerializedSize() > 16 ?
            PX4ActuatorControls::getKeyMaxCdrSerializedSize() : 16;
    m_keyBuffer = reinterpret_cast<unsigned char*>(malloc(keyLength));
    memset(m_keyBuffer, 0, keyLength);
}

PX4ActuatorControlsPubSubType::~PX4ActuatorControlsPubSubType() {
    if (m_keyBuffer != nullptr) {
        free(m_keyBuffer);
    }
}

bool PX4ActuatorControlsPubSubType::serialize(void* data, SerializedPayload_t* payload) {
    PX4ActuatorControls* p_type = static_cast<PX4ActuatorControls*>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->max_size);
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
    payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    
    try {
        ser << *p_type;
    } catch (eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/) {
        return false;
    }

    payload->length = static_cast<uint32_t>(ser.getSerializedDataLength());
    return true;
}

bool PX4ActuatorControlsPubSubType::deserialize(SerializedPayload_t* payload, void* data) {
    try {
        PX4ActuatorControls* p_type = static_cast<PX4ActuatorControls*>(data);
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->length);
        eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
        deser >> *p_type;
    } catch (eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/) {
        return false;
    }
    return true;
}

std::function<uint32_t()> PX4ActuatorControlsPubSubType::getSerializedSizeProvider(void* data) {
    return [data]() -> uint32_t {
        return static_cast<uint32_t>(type::getCdrSerializedSize(*static_cast<PX4ActuatorControls*>(data))) +
                4u /*encapsulation*/;
    };
}

void* PX4ActuatorControlsPubSubType::createData() {
    return reinterpret_cast<void*>(new PX4ActuatorControls());
}

void PX4ActuatorControlsPubSubType::deleteData(void* data) {
    delete(reinterpret_cast<PX4ActuatorControls*>(data));
}

bool PX4ActuatorControlsPubSubType::getKey(void* data, InstanceHandle_t* handle, bool force_md5) {
    if (!m_isGetKeyDefined) {
        return false;
    }

    PX4ActuatorControls* p_type = static_cast<PX4ActuatorControls*>(data);
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(m_keyBuffer),
            PX4ActuatorControls::getKeyMaxCdrSerializedSize());
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS);
    p_type->serializeKey(ser);
    
    if (force_md5 || PX4ActuatorControls::getKeyMaxCdrSerializedSize() > 16) {
        m_md5.init();
        m_md5.update(m_keyBuffer, static_cast<unsigned int>(ser.getSerializedDataLength()));
        m_md5.finalize();
        for (uint8_t i = 0; i < 16; ++i) {
            handle->value[i] = m_md5.digest[i];
        }
    } else {
        for (uint8_t i = 0; i < 16; ++i) {
            handle->value[i] = m_keyBuffer[i];
        }
    }
    return true;
}

}
}
