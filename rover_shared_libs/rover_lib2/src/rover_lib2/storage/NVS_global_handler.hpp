#ifndef ROVER_LIB2_STORAGE_NVS_GLOBAL_HANDLER_HPP
#define ROVER_LIB2_STORAGE_NVS_GLOBAL_HANDLER_HPP

#include "rover_lib2/helpers/assert.hpp"

#include <nvs.h>
#include <nvs_flash.h>

class NVSGlobalHandler
{
  public:
    static void Init(void)
    {
        static NVSGlobalHandler handler;
    }

    NVSGlobalHandler(const NVSGlobalHandler&) = delete;
    NVSGlobalHandler& operator=(const NVSGlobalHandler&) = delete;

  private:
    NVSGlobalHandler()
    {
        esp_err_t err = nvs_flash_init();

        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ASSERT_MSG("NVS partition needs to be reset, persistent data will be lost!");

            if (nvs_flash_erase() != ESP_OK)
            {
                ASSERT_MSG("No NVS partition exists, this should never happen.");
            }

            nvs_flash_init();
        }
    }
};

#endif  // ROVER_LIB2_STORAGE_NVS_GLOBAL_HANDLER_HPP
