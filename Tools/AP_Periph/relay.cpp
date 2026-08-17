#include <AP_HAL/AP_HAL.h> 
#include "AP_Periph.h"
extern const AP_HAL::HAL &hal;
#include <hal.h>

#if AP_PERIPH_RELAY_ENABLED

#include <dronecan_msgs.h>

void AP_Periph_FW::handle_hardpoint_command(CanardInstance* canard_instance, CanardRxTransfer* transfer)


{


    // <<< DEBUG ADD: 函数入口 - 确认消息到达处理函数
#ifdef DEBUG_HARDPOINT
    hal.console->printf("[HARDPOINT DEBUG] Enter handle_hardpoint_command, from node %u\n", transfer->source_node_id);
#endif


    uavcan_equipment_hardpoint_Command cmd {};
       if (uavcan_equipment_hardpoint_Command_decode(transfer, &cmd)) {
        // Failed to decode
          #ifdef DEBUG_HARDPOINT
           hal.console->printf("[HARDPOINT DEBUG] Decode failed!\n");
          #endif
        return;
    }
    #ifdef DEBUG_HARDPOINT
    {
    if (cmd.hardpoint_id == 0)
        hal.gpio->write(50, cmd.command);  // PA8 (GPIO50): 0=低, 1=高
    if (cmd.hardpoint_id == 1)
        hal.gpio->write(51, cmd.command);  // PA8 (GPIO50): 0=低, 1=高
    if (cmd.hardpoint_id == 2)
        hal.gpio->write(52, cmd.command);  // PA8 (GPIO50): 0=低, 1=高
    if (cmd.hardpoint_id == 3)
        hal.gpio->write(53, cmd.command);
    if (cmd.hardpoint_id == 4)
        hal.gpio->write(54, cmd.command);
    if (cmd.hardpoint_id == 5)
        hal.gpio->write(55, cmd.command);
    }
    #endif

    // 增加：打印接收到的 hardpoint_id 和 command 值（核心调试信息）
#ifdef DEBUG_HARDPOINT
    hal.console->printf("[HARDPOINT DEBUG] Received hardpoint_id=%u, command=%u\n", 
                        (unsigned)cmd.hardpoint_id, 
                        (unsigned)cmd.command);
#endif

    // Command must be 0 or 1, other values may be supported in the future
    // rejecting them now ensures no change in behaviour
    if ((cmd.command != 0) && (cmd.command != 1)) {
        return;
    }

    // Translate hardpoint ID to relay function
    AP_Relay_Params::FUNCTION fun;
    switch (cmd.hardpoint_id) {
        case 0 ... 15:
            // 0 to 15 are continuous
            fun = AP_Relay_Params::FUNCTION(cmd.hardpoint_id + (uint8_t)AP_Relay_Params::FUNCTION::DroneCAN_HARDPOINT_0);
            break;

        default:
            // ID not supported
            return;
    }

    // Set relay
    relay.set(fun, cmd.command);

}
#endif
