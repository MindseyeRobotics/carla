// Copyright (c) 2024 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "PX4ActuatorControls.h"
#include <fastcdr/Cdr.h>
#include <fastcdr/exceptions/BadParamException.h>
#include <utility>

using namespace eprosima::fastcdr::exception;

namespace px4_msgs {
namespace msg {

PX4ActuatorControls::PX4ActuatorControls() : m_timestamp(0), m_control() {
    m_control.fill(0.0f);
}

PX4ActuatorControls::~PX4ActuatorControls() {}

PX4ActuatorControls::PX4ActuatorControls(const PX4ActuatorControls& x) {
    m_timestamp = x.m_timestamp;
    m_control = x.m_control;
}

PX4ActuatorControls::PX4ActuatorControls(PX4ActuatorControls&& x) noexcept {
    m_timestamp = x.m_timestamp;
    m_control = std::move(x.m_control);
}

PX4ActuatorControls& PX4ActuatorControls::operator=(const PX4ActuatorControls& x) {
    m_timestamp = x.m_timestamp;
    m_control = x.m_control;
    return *this;
}

PX4ActuatorControls& PX4ActuatorControls::operator=(PX4ActuatorControls&& x) noexcept {
    m_timestamp = x.m_timestamp;
    m_control = std::move(x.m_control);
    return *this;
}

bool PX4ActuatorControls::operator==(const PX4ActuatorControls& x) const {
    return (m_timestamp == x.m_timestamp && m_control == x.m_control);
}

bool PX4ActuatorControls::operator!=(const PX4ActuatorControls& x) const {
    return !(*this == x);
}

size_t PX4ActuatorControls::getMaxCdrSerializedSize(size_t current_alignment) {
    size_t initial_alignment = current_alignment;
    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8); // timestamp
    current_alignment += (8 * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4); // control array
    return current_alignment - initial_alignment;
}

size_t PX4ActuatorControls::getCdrSerializedSize(const PX4ActuatorControls& data, size_t current_alignment) {
    size_t initial_alignment = current_alignment;
    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);
    current_alignment += (8 * 4) + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);
    return current_alignment - initial_alignment;
}

void PX4ActuatorControls::serialize(eprosima::fastcdr::Cdr& scdr) const {
    scdr << m_timestamp;
    scdr << m_control;
}

void PX4ActuatorControls::deserialize(eprosima::fastcdr::Cdr& dcdr) {
    dcdr >> m_timestamp;
    dcdr >> m_control;
}

size_t PX4ActuatorControls::getKeyMaxCdrSerializedSize(size_t current_alignment) {
    return 0;
}

bool PX4ActuatorControls::isKeyDefined() {
    return false;
}

void PX4ActuatorControls::serializeKey(eprosima::fastcdr::Cdr& scdr) const {
    (void) scdr;
}

void PX4ActuatorControls::timestamp(uint64_t _timestamp) {
    m_timestamp = _timestamp;
}

uint64_t PX4ActuatorControls::timestamp() const {
    return m_timestamp;
}

uint64_t& PX4ActuatorControls::timestamp() {
    return m_timestamp;
}

void PX4ActuatorControls::control(const std::array<float, 8>& _control) {
    m_control = _control;
}

void PX4ActuatorControls::control(std::array<float, 8>&& _control) {
    m_control = std::move(_control);
}

const std::array<float, 8>& PX4ActuatorControls::control() const {
    return m_control;
}

std::array<float, 8>& PX4ActuatorControls::control() {
    return m_control;
}

}
}
