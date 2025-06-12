#ifndef ROVER_LIB2_STORAGE_NVS_DATA_HANDLE_HPP
#define ROVER_LIB2_STORAGE_NVS_DATA_HANDLE_HPP

#include "NVS_global_handler.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/assert.hpp"

#include <cstring>

DEFINE_LOG_NODE(NVSDataHandle, Logger::eNodeState::ON);

template<typename Data_T>
class NVSDataHandle
{
    static_assert(
        std::is_same_v<
            int8_t,
            Data_T> || std::is_same_v<uint8_t, Data_T> || std::is_same_v<int16_t, Data_T> || std::is_same_v<uint16_t, Data_T> || std::is_same_v<int32_t, Data_T> || std::is_same_v<uint32_t, Data_T> || std::is_same_v<int64_t, Data_T> || std::is_same_v<uint64_t, Data_T>,
        "Type not supported");

    static constexpr size_t NVS_MAX_LENGTH_STR = 15UL;

  public:
    /**
     * @brief
     * @param namespace_ Isn't copied so must be valid until handle's end of life
     * @param key_ Isn't copied so must be valid until handle's end of life
     * @param defaultValue_ Used temporarily until the NVS data can be accessed
     */
    NVSDataHandle(const char* namespace_, const char* key_, Data_T defaultValue_ = 0UL):
        _namespace(namespace_),
        _key(key_),
        _defaultValue(defaultValue_),
        _dataInSync(false),
        _currentValue(defaultValue_)
    {
        ASSERT_COND_MSG_ARGS(std::strlen(namespace_) <= NVS_MAX_LENGTH_STR,
                             "NVS storage namespace must can't be more than %u chars",
                             NVS_MAX_LENGTH_STR);
        ASSERT_COND_MSG(std::strcmp(namespace_, "") != 0, "NVS storage namespace can't be empty");

        ASSERT_COND_MSG_ARGS(std::strlen(key_) <= NVS_MAX_LENGTH_STR,
                             "NVS storage name must can't be more than %u chars",
                             NVS_MAX_LENGTH_STR);
        ASSERT_COND_MSG(std::strcmp(key_, "") != 0, "NVS storage name can't be empty");

        NVSGlobalHandler::Init();

        esp_err_t err = nvs_open(_namespace, nvs_open_mode_t::NVS_READWRITE, &_nvsHandle);
        ASSERT_COND_MSG_ARGS(err == ESP_OK, "Failed to open nvs namespace \"%s\" with err: %s", _namespace, esp_err_to_name(err));

        this->getValue();  // Tries to sync value
        if (!_dataInSync)
        {
            LOG_WARN(Logger::Nodes::NVSDataHandle, "Problem accessing wanted NVS value, defaulting to default value instead");
        }
    }

    NVSDataHandle(const NVSDataHandle&) = delete;
    NVSDataHandle& operator=(const NVSDataHandle&) = delete;
    NVSDataHandle(NVSDataHandle&&) = delete;
    NVSDataHandle& operator=(NVSDataHandle&&) = delete;

    ~NVSDataHandle()
    {
        nvs_close(_nvsHandle);
    }

    /**
     * @brief
     * @param value_ Will not be overwritten on failure
     */
    Data_T getValue(void)
    {
        if (_dataInSync)
        {
            return _currentValue;
        }

        Data_T retVal = static_cast<Data_T>(0);
        esp_err_t err = ESP_OK;

        if constexpr (std::is_same_v<Data_T, int8_t>)
        {
            err = nvs_get_i8(_nvsHandle, _key, &retVal);
        }
        else if constexpr (std::is_same_v<Data_T, uint8_t>)
        {
            err = nvs_get_u8(_nvsHandle, _key, &retVal);
        }
        else if constexpr (std::is_same_v<Data_T, int16_t>)
        {
            err = nvs_get_i16(_nvsHandle, _key, &retVal);
        }
        else if constexpr (std::is_same_v<Data_T, uint16_t>)
        {
            err = nvs_get_u16(_nvsHandle, _key, &retVal);
        }
        else if constexpr (std::is_same_v<Data_T, int32_t>)
        {
            err = nvs_get_i32(_nvsHandle, _key, &retVal);
        }
        else if constexpr (std::is_same_v<Data_T, uint32_t>)
        {
            err = nvs_get_u32(_nvsHandle, _key, &retVal);
        }
        else if constexpr (std::is_same_v<Data_T, int64_t>)
        {
            err = nvs_get_i64(_nvsHandle, _key, &retVal);
        }
        else if constexpr (std::is_same_v<Data_T, uint64_t>)
        {
            err = nvs_get_u64(_nvsHandle, _key, &retVal);
        }
        // TODO: Add blob support for compatibility with any datatype

        if (err == ESP_OK)
        {
            _currentValue = retVal;
            _dataInSync = true;
            return _currentValue;
        }
        else
        {
            logGetSetError(err, _key);
            return _defaultValue;
        }
    }

    bool writeValue(const Data_T& value_)
    {
        if (_dataInSync && _currentValue == value_)
        {
            return true;
        }

        esp_err_t err = ESP_OK;
        if constexpr (std::is_same_v<Data_T, int8_t>)
        {
            err = nvs_set_i8(_nvsHandle, _key, value_);
        }
        else if constexpr (std::is_same_v<Data_T, uint8_t>)
        {
            err = nvs_set_u8(_nvsHandle, _key, value_);
        }
        else if constexpr (std::is_same_v<Data_T, int16_t>)
        {
            err = nvs_set_i16(_nvsHandle, _key, value_);
        }
        else if constexpr (std::is_same_v<Data_T, uint16_t>)
        {
            err = nvs_set_u16(_nvsHandle, _key, value_);
        }
        else if constexpr (std::is_same_v<Data_T, int32_t>)
        {
            err = nvs_set_i32(_nvsHandle, _key, value_);
        }
        else if constexpr (std::is_same_v<Data_T, uint32_t>)
        {
            err = nvs_set_u32(_nvsHandle, _key, value_);
        }
        else if constexpr (std::is_same_v<Data_T, int64_t>)
        {
            err = nvs_set_i64(_nvsHandle, _key, value_);
        }
        else if constexpr (std::is_same_v<Data_T, uint64_t>)
        {
            err = nvs_set_u64(_nvsHandle, _key, value_);
        }
        // TODO: Add blob support for compatibility with any datatype

        if (err != ESP_OK)
        {
            logGetSetError(err, _key);
            return false;
        }

        err = nvs_commit(_nvsHandle);
        if (err != ESP_OK)
        {
            logGetSetError(err, _key);
            return false;
        }

        if (!_dataInSync)
        {
            _dataInSync = true;
        }

        _currentValue = value_;
        return true;
    }

    bool dataInSync(void)
    {
        return _dataInSync;
    }

  private:
    void logGetSetError(const esp_err_t& err_, const char* key_)
    {
        switch (err_)
        {
            case ESP_FAIL:
                ASSERT_MSG("Couldn't access NVS data, partition most likely corrupted...");
                break;

            case ESP_ERR_NVS_NOT_FOUND:
                LOG_WARN(Logger::Nodes::NVSDataHandle,
                         "Couldn't access NVS data, no data found at key: \"%s\". Data needs to be written to first",
                         key_);
                break;

            case ESP_ERR_NVS_INVALID_HANDLE:
                LOG_WARN(Logger::Nodes::NVSDataHandle,
                         "Couldn't access NVS data, nvs handle isn't valid, check earlier logs for info");
                break;

            case ESP_ERR_NVS_INVALID_NAME:
                LOG_WARN(Logger::Nodes::NVSDataHandle,
                         "Couldn't access NVS data, key: \"%s\" doesn't satisfy NVS constraints",
                         key_);
                break;

            case ESP_ERR_NVS_READ_ONLY:
                LOG_WARN(Logger::Nodes::NVSDataHandle, "Couldn't write NVS data \"%s\", space was opened as read-only", key_);
                break;

            case ESP_ERR_NVS_NOT_ENOUGH_SPACE:
                [[fallthrough]];
            case ESP_ERR_NVS_INVALID_LENGTH:
                LOG_WARN(Logger::Nodes::NVSDataHandle,
                         "Couldn't access NVS data, space's length is not sufficient to store requested data");
                break;

            case ESP_ERR_NVS_REMOVE_FAILED:
                LOG_WARN(Logger::Nodes::NVSDataHandle,
                         "Couldn't write NVS data \"%s\", writer failed, should still be applied during next commit",
                         key_);
                break;

            default:
                LOG_WARN(Logger::Nodes::NVSDataHandle, "Shouldn't fall here, implementation error");
                break;
        }
    }

    const char* _namespace;
    const char* _key;
    const Data_T _defaultValue;

    bool _dataInSync;
    Data_T _currentValue;
    nvs_handle_t _nvsHandle;
};

#endif  // ROVER_LIB2_STORAGE_NVS_DATA_HANDLE_HPP
