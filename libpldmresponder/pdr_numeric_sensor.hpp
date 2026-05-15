#pragma once

#include "libpldmresponder/pdr_utils.hpp"

#include <libpldm/platform.h>

#include <phosphor-logging/lg2.hpp>

PHOSPHOR_LOG2_USING;

namespace pldm
{
namespace responder
{
namespace pdr_numeric_sensor
{
using Json = nlohmann::json;

static const Json empty{};

/** @brief Parse PDR JSON file and generate numeric sensor PDR structure
 *
 *  @param[in] json - the JSON Object with the numeric sensor PDR
 *  @param[out] handler - the Parser of PLDM command handler
 *  @param[out] repo - pdr::RepoInterface
 *
 */
template <class DBusInterface, class Handler>
void generateNumericSensorPDR(const DBusInterface& dBusIntf, const Json& json,
                               Handler& handler,
                               pdr_utils::RepoInterface& repo,
                               pldm_entity_association_tree* /*bmcEntityTree*/)
{
    static const std::vector<Json> emptyList{};
    auto entries = json.value("entries", emptyList);
    for (const auto& e : entries)
    {
        std::vector<uint8_t> entry{};
        entry.resize(sizeof(pldm_numeric_sensor_value_pdr));

        pldm_numeric_sensor_value_pdr* pdr =
            reinterpret_cast<pldm_numeric_sensor_value_pdr*>(entry.data());
        if (!pdr)
        {
            error("Failed to get numeric sensor PDR.");
            continue;
        }
        pdr->hdr.record_handle = 0;
        pdr->hdr.version = 1;
        pdr->hdr.type = PLDM_NUMERIC_SENSOR_PDR;
        pdr->hdr.record_change_num = 0;
        pdr->hdr.length =
            sizeof(pldm_numeric_sensor_value_pdr) - sizeof(pldm_pdr_hdr);

        pdr->terminus_handle = e.value("terminus_handle", 0);

        try
        {
            std::string entity_path = e.value("entity_path", "");
            auto& associatedEntityMap = handler.getAssociateEntityMap();
            if (entity_path != "" && associatedEntityMap.contains(entity_path))
            {
                pdr->entity_type =
                    associatedEntityMap.at(entity_path).entity_type;
                pdr->entity_instance =
                    associatedEntityMap.at(entity_path).entity_instance_num;
                pdr->container_id =
                    associatedEntityMap.at(entity_path).entity_container_id;
            }
            else
            {
                pdr->entity_type = e.value("type", 0);
                pdr->entity_instance = e.value("instance", 0);
                pdr->container_id = e.value("container", 0);

                // do not create the PDR when the FRU or the entity path is not
                // present
                if (!pdr->entity_type)
                {
                    continue;
                }
            }
        }
        catch (const std::exception&)
        {
            pdr->entity_type = e.value("type", 0);
            pdr->entity_instance = e.value("instance", 0);
            pdr->container_id = e.value("container", 0);
        }

        pdr->sensor_init = e.value("sensor_init", PLDM_NO_INIT);
        pdr->sensor_auxiliary_names_pdr =
            e.value("sensor_auxiliary_names_pdr", false);
        pdr->base_unit = e.value("base_unit", 0);
        pdr->unit_modifier = e.value("unit_modifier", 0);
        pdr->rate_unit = e.value("rate_unit", 0);
        pdr->base_oem_unit_handle = e.value("base_oem_unit_handle", 0);
        pdr->aux_unit = e.value("aux_unit", 0);
        pdr->aux_unit_modifier = e.value("aux_unit_modifier", 0);
        pdr->aux_rate_unit = e.value("aux_rate_unit", 0);
        pdr->rel = e.value("rel", 0);
        pdr->aux_oem_unit_handle = e.value("aux_oem_unit_handle", 0);
        pdr->is_linear = e.value("is_linear", true);
        pdr->sensor_data_size =
            e.value("sensor_data_size", PLDM_SENSOR_DATA_SIZE_UINT32);
        pdr->resolution = e.value("resolution", 1.0);
        pdr->offset = e.value("offset", 0.0);
        pdr->accuracy = e.value("accuracy", 0);
        pdr->plus_tolerance = e.value("plus_tolerance", 0);
        pdr->minus_tolerance = e.value("minus_tolerance", 0);
        // Set hysteresis based on sensor_data_size
        uint32_t hysteresisValue = e.value("hysteresis", 0);
        switch (pdr->sensor_data_size)
        {
            case PLDM_SENSOR_DATA_SIZE_UINT8:
                pdr->hysteresis.value_u8 = static_cast<uint8_t>(hysteresisValue);
                break;
            case PLDM_SENSOR_DATA_SIZE_SINT8:
                pdr->hysteresis.value_s8 = static_cast<int8_t>(hysteresisValue);
                break;
            case PLDM_SENSOR_DATA_SIZE_UINT16:
                pdr->hysteresis.value_u16 = static_cast<uint16_t>(hysteresisValue);
                break;
            case PLDM_SENSOR_DATA_SIZE_SINT16:
                pdr->hysteresis.value_s16 = static_cast<int16_t>(hysteresisValue);
                break;
            case PLDM_SENSOR_DATA_SIZE_UINT32:
                pdr->hysteresis.value_u32 = hysteresisValue;
                break;
            case PLDM_SENSOR_DATA_SIZE_SINT32:
                pdr->hysteresis.value_s32 = static_cast<int32_t>(hysteresisValue);
                break;
        }
        pdr->supported_thresholds.byte = e.value("supported_thresholds", 0);
        pdr->threshold_and_hysteresis_volatility.byte =
            e.value("threshold_and_hysteresis_volatility", 0);
        pdr->state_transition_interval =
            e.value("state_transition_interval", 0.0);
        pdr->update_interval = e.value("update_interval", 0.0);

        // Set max/min readable based on sensor_data_size
        switch (pdr->sensor_data_size)
        {
            case PLDM_SENSOR_DATA_SIZE_UINT8:
                pdr->max_readable.value_u8 = e.value("max_readable", 255);
                pdr->min_readable.value_u8 = e.value("min_readable", 0);
                break;
            case PLDM_SENSOR_DATA_SIZE_SINT8:
                pdr->max_readable.value_s8 = e.value("max_readable", 127);
                pdr->min_readable.value_s8 = e.value("min_readable", -128);
                break;
            case PLDM_SENSOR_DATA_SIZE_UINT16:
                pdr->max_readable.value_u16 = e.value("max_readable", 65535);
                pdr->min_readable.value_u16 = e.value("min_readable", 0);
                break;
            case PLDM_SENSOR_DATA_SIZE_SINT16:
                pdr->max_readable.value_s16 = e.value("max_readable", 32767);
                pdr->min_readable.value_s16 = e.value("min_readable", -32768);
                break;
            case PLDM_SENSOR_DATA_SIZE_UINT32:
                pdr->max_readable.value_u32 =
                    e.value("max_readable", 0xFFFFFFFF);
                pdr->min_readable.value_u32 = e.value("min_readable", 0);
                break;
            case PLDM_SENSOR_DATA_SIZE_SINT32:
                pdr->max_readable.value_s32 =
                    e.value("max_readable", 0x7FFFFFFF);
                pdr->min_readable.value_s32 =
                    e.value("min_readable", static_cast<int32_t>(0x80000000));
                break;
            default:
                break;
        }

        pdr->range_field_format =
            e.value("range_field_format", PLDM_RANGE_FIELD_FORMAT_UINT32);
        pdr->range_field_support.byte = e.value("range_field_support", 0);

        // Set range fields based on range_field_format
        switch (pdr->range_field_format)
        {
            case PLDM_RANGE_FIELD_FORMAT_UINT8:
                pdr->nominal_value.value_u8 = e.value("nominal_value", 0);
                pdr->normal_max.value_u8 = e.value("normal_max", 0);
                pdr->normal_min.value_u8 = e.value("normal_min", 0);
                pdr->warning_high.value_u8 = e.value("warning_high", 0);
                pdr->warning_low.value_u8 = e.value("warning_low", 0);
                pdr->critical_high.value_u8 = e.value("critical_high", 0);
                pdr->critical_low.value_u8 = e.value("critical_low", 0);
                pdr->fatal_high.value_u8 = e.value("fatal_high", 0);
                pdr->fatal_low.value_u8 = e.value("fatal_low", 0);
                break;
            case PLDM_RANGE_FIELD_FORMAT_SINT8:
                pdr->nominal_value.value_s8 = e.value("nominal_value", 0);
                pdr->normal_max.value_s8 = e.value("normal_max", 0);
                pdr->normal_min.value_s8 = e.value("normal_min", 0);
                pdr->warning_high.value_s8 = e.value("warning_high", 0);
                pdr->warning_low.value_s8 = e.value("warning_low", 0);
                pdr->critical_high.value_s8 = e.value("critical_high", 0);
                pdr->critical_low.value_s8 = e.value("critical_low", 0);
                pdr->fatal_high.value_s8 = e.value("fatal_high", 0);
                pdr->fatal_low.value_s8 = e.value("fatal_low", 0);
                break;
            case PLDM_RANGE_FIELD_FORMAT_UINT16:
                pdr->nominal_value.value_u16 = e.value("nominal_value", 0);
                pdr->normal_max.value_u16 = e.value("normal_max", 0);
                pdr->normal_min.value_u16 = e.value("normal_min", 0);
                pdr->warning_high.value_u16 = e.value("warning_high", 0);
                pdr->warning_low.value_u16 = e.value("warning_low", 0);
                pdr->critical_high.value_u16 = e.value("critical_high", 0);
                pdr->critical_low.value_u16 = e.value("critical_low", 0);
                pdr->fatal_high.value_u16 = e.value("fatal_high", 0);
                pdr->fatal_low.value_u16 = e.value("fatal_low", 0);
                break;
            case PLDM_RANGE_FIELD_FORMAT_SINT16:
                pdr->nominal_value.value_s16 = e.value("nominal_value", 0);
                pdr->normal_max.value_s16 = e.value("normal_max", 0);
                pdr->normal_min.value_s16 = e.value("normal_min", 0);
                pdr->warning_high.value_s16 = e.value("warning_high", 0);
                pdr->warning_low.value_s16 = e.value("warning_low", 0);
                pdr->critical_high.value_s16 = e.value("critical_high", 0);
                pdr->critical_low.value_s16 = e.value("critical_low", 0);
                pdr->fatal_high.value_s16 = e.value("fatal_high", 0);
                pdr->fatal_low.value_s16 = e.value("fatal_low", 0);
                break;
            case PLDM_RANGE_FIELD_FORMAT_UINT32:
                pdr->nominal_value.value_u32 = e.value("nominal_value", 0);
                pdr->normal_max.value_u32 = e.value("normal_max", 0);
                pdr->normal_min.value_u32 = e.value("normal_min", 0);
                pdr->warning_high.value_u32 = e.value("warning_high", 0);
                pdr->warning_low.value_u32 = e.value("warning_low", 0);
                pdr->critical_high.value_u32 = e.value("critical_high", 0);
                pdr->critical_low.value_u32 = e.value("critical_low", 0);
                pdr->fatal_high.value_u32 = e.value("fatal_high", 0);
                pdr->fatal_low.value_u32 = e.value("fatal_low", 0);
                break;
            case PLDM_RANGE_FIELD_FORMAT_SINT32:
                pdr->nominal_value.value_s32 = e.value("nominal_value", 0);
                pdr->normal_max.value_s32 = e.value("normal_max", 0);
                pdr->normal_min.value_s32 = e.value("normal_min", 0);
                pdr->warning_high.value_s32 = e.value("warning_high", 0);
                pdr->warning_low.value_s32 = e.value("warning_low", 0);
                pdr->critical_high.value_s32 = e.value("critical_high", 0);
                pdr->critical_low.value_s32 = e.value("critical_low", 0);
                pdr->fatal_high.value_s32 = e.value("fatal_high", 0);
                pdr->fatal_low.value_s32 = e.value("fatal_low", 0);
                break;
            case PLDM_RANGE_FIELD_FORMAT_REAL32:
                pdr->nominal_value.value_f32 = e.value("nominal_value", 0.0);
                pdr->normal_max.value_f32 = e.value("normal_max", 0.0);
                pdr->normal_min.value_f32 = e.value("normal_min", 0.0);
                pdr->warning_high.value_f32 = e.value("warning_high", 0.0);
                pdr->warning_low.value_f32 = e.value("warning_low", 0.0);
                pdr->critical_high.value_f32 = e.value("critical_high", 0.0);
                pdr->critical_low.value_f32 = e.value("critical_low", 0.0);
                pdr->fatal_high.value_f32 = e.value("fatal_high", 0.0);
                pdr->fatal_low.value_f32 = e.value("fatal_low", 0.0);
                break;
            default:
                break;
        }

        auto dbusEntry = e.value("dbus", empty);
        auto objectPath = dbusEntry.value("path", "");
        auto interface = dbusEntry.value("interface", "");
        auto propertyName = dbusEntry.value("property_name", "");
        auto propertyType = dbusEntry.value("property_type", "");

        pldm::responder::pdr_utils::DbusMappings dbusMappings{};
        pldm::responder::pdr_utils::DbusValMaps dbusValMaps{};
        pldm::utils::DBusMapping dbusMapping{};
        try
        {
            auto service =
                dBusIntf.getService(objectPath.c_str(), interface.c_str());

            dbusMapping = pldm::utils::DBusMapping{objectPath, interface,
                                                   propertyName, propertyType};
        }
        catch (const std::exception& e)
        {
            error(
                "D-Bus object path does not exist for sensor ID '{SENSOR_ID}', error - {ERROR}",
                "SENSOR_ID", static_cast<uint16_t>(pdr->sensor_id), "ERROR",
                e);
        }
        dbusMappings.emplace_back(std::move(dbusMapping));
        pdr->sensor_id = handler.getNextSensorId();
        handler.addDbusObjMaps(
            pdr->sensor_id,
            std::make_tuple(std::move(dbusMappings), std::move(dbusValMaps)),
            pdr_utils::TypeId::PLDM_SENSOR_ID);

        pdr_utils::PdrEntry pdrEntry{};
        pdrEntry.data = entry.data();
        pdrEntry.size = sizeof(pldm_numeric_sensor_value_pdr);
        repo.addRecord(pdrEntry);
    }
}

} // namespace pdr_numeric_sensor
} // namespace responder
} // namespace pldm
