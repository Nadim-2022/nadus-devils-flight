#ifndef DEVICEID_HPP
#define DEVICEID_HPP

#include <pico/unique_id.h>

#include <string>

namespace Config
{

/**
 * @brief Gets the Device ID of the Raspberry Pico.
 *
 * @return std::string The string representation of the Device ID.
 */
inline std::string getDeviceId()
{
    std::string id;
    id.resize(2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    pico_get_unique_board_id_string(id.data(), id.size() + 1);

    return id;
}

} // namespace Config

#endif /* DEVICEID_HPP */
