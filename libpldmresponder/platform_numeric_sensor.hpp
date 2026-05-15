#pragma once

#include "common/utils.hpp"
#include "libpldmresponder/pdr_utils.hpp"

#include <libpldm/platform.h>

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace pldm
{
namespace responder
{
namespace platform_numeric_sensor
{

/** @brief Handler for GetSensorReading command for numeric sensors
 *
 *  @param[in] request - Request message
 *  @param[in] payloadLength - Request message payload length
 *  @param[in] handler - Reference to handler
 *  @return Response - PLDM Response message
 */
template <class Handler>
Response getSensorReading(const pldm_msg* request, size_t payloadLength,
                         Handler& handler)
{
    Response response(sizeof(pldm_msg_hdr) +
                     PLDM_GET_SENSOR_READING_MIN_RESP_BYTES);
    auto responsePtr = reinterpret_cast<pldm_msg*>(response.data());

    if (payloadLength != PLDM_GET_SENSOR_READING_REQ_BYTES)
    {
        return CmdHandler::ccOnlyResponse(request, PLDM_ERROR_INVALID_LENGTH);
    }

    uint16_t sensorId{};
    bool8_t rearmEventState{};
    auto rc = decode_get_sensor_reading_req(request, payloadLength, &sensorId,
                                           &rearmEventState);
    if (rc != PLDM_SUCCESS)
    {
        return CmdHandler::ccOnlyResponse(request, rc);
    }

    // Get D-Bus mapping for this sensor
    try
    {
        const auto& [dbusMappings, dbusValMaps] = handler.getDbusObjMaps(
            sensorId, pdr_utils::TypeId::PLDM_SENSOR_ID);

        if (dbusMappings.empty())
        {
            return CmdHandler::ccOnlyResponse(
                request, PLDM_PLATFORM_INVALID_SENSOR_ID);
        }

        const auto& dbusMapping = dbusMappings[0];

        // Read the property value from D-Bus
        auto value = pldm::utils::DBusHandler().getDbusPropertyVariant(
            dbusMapping.objectPath.c_str(), dbusMapping.propertyName.c_str(),
            dbusMapping.interface.c_str());

        // Set sensor operational state and event states
        uint8_t sensorDataSize = PLDM_SENSOR_DATA_SIZE_UINT32;
        uint8_t sensorOperationalState = PLDM_SENSOR_ENABLED;
        uint8_t sensorEventMessageEnable = PLDM_NO_EVENT_GENERATION;
        uint8_t presentState = PLDM_SENSOR_NORMAL;
        uint8_t previousState = PLDM_SENSOR_NORMAL;
        uint8_t eventState = PLDM_SENSOR_NORMAL;

        union_sensor_data_size presentReading;
        std::memset(&presentReading, 0, sizeof(presentReading));

        // Convert variant to appropriate sensor data type
        if (dbusMapping.propertyType == "uint32_t")
        {
            presentReading.value_u32 = std::get<uint32_t>(value);
            sensorDataSize = PLDM_SENSOR_DATA_SIZE_UINT32;
        }
        else if (dbusMapping.propertyType == "int32_t")
        {
            presentReading.value_s32 = std::get<int32_t>(value);
            sensorDataSize = PLDM_SENSOR_DATA_SIZE_SINT32;
        }
        else if (dbusMapping.propertyType == "uint16_t")
        {
            presentReading.value_u16 = std::get<uint16_t>(value);
            sensorDataSize = PLDM_SENSOR_DATA_SIZE_UINT16;
        }
        else if (dbusMapping.propertyType == "int16_t")
        {
            presentReading.value_s16 = std::get<int16_t>(value);
            sensorDataSize = PLDM_SENSOR_DATA_SIZE_SINT16;
        }
        else if (dbusMapping.propertyType == "uint8_t")
        {
            presentReading.value_u8 = std::get<uint8_t>(value);
            sensorDataSize = PLDM_SENSOR_DATA_SIZE_UINT8;
        }
        else if (dbusMapping.propertyType == "double")
        {
            // For double, convert to int32_t
            double doubleValue = std::get<double>(value);
            presentReading.value_s32 = static_cast<int32_t>(doubleValue);
            sensorDataSize = PLDM_SENSOR_DATA_SIZE_SINT32;
        }
        else
        {
            error(
                "Unsupported property type '{TYPE}' for sensor ID '{SENSOR_ID}'",
                "TYPE", dbusMapping.propertyType, "SENSOR_ID", sensorId);
            return CmdHandler::ccOnlyResponse(request, PLDM_ERROR);
        }

        rc = encode_get_sensor_reading_resp(
            request->hdr.instance_id, PLDM_SUCCESS, sensorDataSize,
            sensorOperationalState, sensorEventMessageEnable, presentState,
            previousState, eventState,
            reinterpret_cast<const uint8_t*>(&presentReading), responsePtr,
            response.size() - sizeof(pldm_msg_hdr));

        if (rc != PLDM_SUCCESS)
        {
            return CmdHandler::ccOnlyResponse(request, rc);
        }

        return response;
    }
    catch (const std::out_of_range& e)
    {
        error("Sensor ID '{SENSOR_ID}' not found in mapping: {ERROR}",
              "SENSOR_ID", sensorId, "ERROR", e);
        return CmdHandler::ccOnlyResponse(request,
                                         PLDM_PLATFORM_INVALID_SENSOR_ID);
    }
    catch (const std::exception& e)
    {
        error("Error reading sensor '{SENSOR_ID}': {ERROR}", "SENSOR_ID",
              sensorId, "ERROR", e);
        return CmdHandler::ccOnlyResponse(request, PLDM_ERROR);
    }
}

} // namespace platform_numeric_sensor
} // namespace responder
} // namespace pldm