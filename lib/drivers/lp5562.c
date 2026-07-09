#include "lp5562.h"
#include <core/common_defines.h>
#include "lp5562_reg.h"
#include <furi_hal.h>

bool lp5562_reset(const FuriHalI2cBusHandle* handle) {
    Reg0D_Reset reg = {.value = 0xFF};
    return furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x0D, *(uint8_t*)&reg, LP5562_I2C_TIMEOUT);
}

bool lp5562_configure(const FuriHalI2cBusHandle* handle) {
    bool result = true;
    Reg08_Config config = {.INT_CLK_EN = true, .PS_EN = true, .PWM_HF = true};
    result &= furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x08, *(uint8_t*)&config, LP5562_I2C_TIMEOUT);

    Reg70_LedMap map = {
        .red = EngSelectI2C,
        .green = EngSelectI2C,
        .blue = EngSelectI2C,
        .white = EngSelectI2C,
    };
    result &= furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x70, *(uint8_t*)&map, LP5562_I2C_TIMEOUT);
    return result;
}

bool lp5562_enable(const FuriHalI2cBusHandle* handle) {
    Reg00_Enable reg = {.CHIP_EN = true, .LOG_EN = true};
    bool result = furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x00, *(uint8_t*)&reg, LP5562_I2C_TIMEOUT);
    //>488μs delay is required after writing to 0x00 register, otherwise program engine will not work
    furi_delay_us(500);
    return result;
}

bool lp5562_set_channel_current(
    const FuriHalI2cBusHandle* handle,
    LP5562Channel channel,
    uint8_t value) {
    uint8_t reg_no;
    if(channel == LP5562ChannelRed) {
        reg_no = LP5562_CHANNEL_RED_CURRENT_REGISTER;
    } else if(channel == LP5562ChannelGreen) {
        reg_no = LP5562_CHANNEL_GREEN_CURRENT_REGISTER;
    } else if(channel == LP5562ChannelBlue) {
        reg_no = LP5562_CHANNEL_BLUE_CURRENT_REGISTER;
    } else if(channel == LP5562ChannelWhite) {
        reg_no = LP5562_CHANNEL_WHITE_CURRENT_REGISTER;
    } else {
        return false;
    }
    return furi_hal_i2c_write_reg_8(handle, LP5562_ADDRESS, reg_no, value, LP5562_I2C_TIMEOUT);
}

bool lp5562_set_channel_value(
    const FuriHalI2cBusHandle* handle,
    LP5562Channel channel,
    uint8_t value) {
    uint8_t reg_no;
    if(channel == LP5562ChannelRed) {
        reg_no = LP5562_CHANNEL_RED_VALUE_REGISTER;
    } else if(channel == LP5562ChannelGreen) {
        reg_no = LP5562_CHANNEL_GREEN_VALUE_REGISTER;
    } else if(channel == LP5562ChannelBlue) {
        reg_no = LP5562_CHANNEL_BLUE_VALUE_REGISTER;
    } else if(channel == LP5562ChannelWhite) {
        reg_no = LP5562_CHANNEL_WHITE_VALUE_REGISTER;
    } else {
        return false;
    }
    return furi_hal_i2c_write_reg_8(handle, LP5562_ADDRESS, reg_no, value, LP5562_I2C_TIMEOUT);
}

uint8_t lp5562_get_channel_value(const FuriHalI2cBusHandle* handle, LP5562Channel channel) {
    uint8_t reg_no;
    uint8_t value = 0;
    if(channel == LP5562ChannelRed) {
        reg_no = LP5562_CHANNEL_RED_VALUE_REGISTER;
    } else if(channel == LP5562ChannelGreen) {
        reg_no = LP5562_CHANNEL_GREEN_VALUE_REGISTER;
    } else if(channel == LP5562ChannelBlue) {
        reg_no = LP5562_CHANNEL_BLUE_VALUE_REGISTER;
    } else if(channel == LP5562ChannelWhite) {
        reg_no = LP5562_CHANNEL_WHITE_VALUE_REGISTER;
    } else {
        return 0;
    }
    furi_hal_i2c_read_reg_8(handle, LP5562_ADDRESS, reg_no, &value, LP5562_I2C_TIMEOUT);
    return value;
}

bool lp5562_set_channel_src(
    const FuriHalI2cBusHandle* handle,
    LP5562Channel channel,
    LP5562Engine src) {
    uint8_t reg_val = 0;
    uint8_t bit_offset = 0;
    bool result = true;

    do {
        if(channel & LP5562ChannelRed) {
            bit_offset = 4;
            channel &= ~LP5562ChannelRed;
        } else if(channel & LP5562ChannelGreen) {
            bit_offset = 2;
            channel &= ~LP5562ChannelGreen;
        } else if(channel & LP5562ChannelBlue) {
            bit_offset = 0;
            channel &= ~LP5562ChannelBlue;
        } else if(channel & LP5562ChannelWhite) {
            bit_offset = 6;
            channel &= ~LP5562ChannelWhite;
        } else {
            return result;
        }

        result &= furi_hal_i2c_read_reg_8(
            handle, LP5562_ADDRESS, 0x70, &reg_val, LP5562_I2C_TIMEOUT);
        reg_val &= ~(0x3 << bit_offset);
        reg_val |= ((src & 0x03) << bit_offset);
        result &= furi_hal_i2c_write_reg_8(
            handle, LP5562_ADDRESS, 0x70, reg_val, LP5562_I2C_TIMEOUT);
    } while(channel != 0);
    return result;
}

bool lp5562_execute_program(
    const FuriHalI2cBusHandle* handle,
    LP5562Engine eng,
    LP5562Channel ch,
    uint16_t* program) {
    if((eng < LP5562Engine1) || (eng > LP5562Engine3)) return false;
    uint8_t reg_val = 0;
    uint8_t bit_offset = 0;
    uint8_t enable_reg = 0;
    bool result = true;

    // Read old value of enable register
    result &= furi_hal_i2c_read_reg_8(
        handle, LP5562_ADDRESS, 0x00, &enable_reg, LP5562_I2C_TIMEOUT);

    // Engine configuration
    bit_offset = (3 - eng) * 2;
    result &= furi_hal_i2c_read_reg_8(
        handle, LP5562_ADDRESS, 0x01, &reg_val, LP5562_I2C_TIMEOUT);
    reg_val &= ~(0x3 << bit_offset);
    reg_val |= (0x01 << bit_offset); // load
    result &= furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x01, reg_val, LP5562_I2C_TIMEOUT);
    furi_delay_us(100);

    // Program load
    for(uint8_t i = 0; i < 16; i++) {
        // Program words are big-endian, so reverse byte order before loading
        program[i] = __REV16(program[i]);
    }
    result &= furi_hal_i2c_write_mem(
        handle,
        LP5562_ADDRESS,
        0x10 + (0x20 * (eng - 1)),
        (uint8_t*)program,
        16 * 2,
        LP5562_I2C_TIMEOUT);

    // Program start
    bit_offset = (3 - eng) * 2;
    result &= furi_hal_i2c_read_reg_8(
        handle, LP5562_ADDRESS, 0x01, &reg_val, LP5562_I2C_TIMEOUT);
    reg_val &= ~(0x3 << bit_offset);
    reg_val |= (0x02 << bit_offset); // run
    result &= furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x01, reg_val, LP5562_I2C_TIMEOUT);

    // Switch output to Execution Engine
    result &= lp5562_set_channel_src(handle, ch, eng);

    enable_reg &= ~(0x3 << bit_offset);
    enable_reg |= (0x02 << bit_offset); // run
    result &= furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x00, enable_reg, LP5562_I2C_TIMEOUT);

    return result;
}

bool lp5562_stop_program(const FuriHalI2cBusHandle* handle, LP5562Engine eng) {
    if((eng < LP5562Engine1) || (eng > LP5562Engine3)) return false;
    uint8_t reg_val = 0;
    uint8_t bit_offset = 0;
    bool result = true;

    // Engine configuration
    bit_offset = (3 - eng) * 2;
    result &= furi_hal_i2c_read_reg_8(
        handle, LP5562_ADDRESS, 0x01, &reg_val, LP5562_I2C_TIMEOUT);
    reg_val &= ~(0x3 << bit_offset);
    // Not setting lowest 2 bits here
    result &= furi_hal_i2c_write_reg_8(
        handle, LP5562_ADDRESS, 0x01, reg_val, LP5562_I2C_TIMEOUT);
    return result;
}

bool lp5562_execute_ramp(
    const FuriHalI2cBusHandle* handle,
    LP5562Engine eng,
    LP5562Channel ch,
    uint8_t val_start,
    uint8_t val_end,
    uint16_t time) {
    if(val_start == val_end) return false;
    bool result = true;

    // Temporary switch to constant value from register
    result &= lp5562_set_channel_src(handle, ch, LP5562Direct);

    // Prepare command sequence
    uint16_t program[16];
    uint8_t diff = (val_end > val_start) ? (val_end - val_start) : (val_start - val_end);
    if(diff == 0) { // Making division below safer
        diff = 1;
    }
    uint16_t time_step = time * 2 / diff;
    uint8_t prescaller = 0;
    if(time_step > 0x3F) {
        time_step /= 32;
        prescaller = 1;
    }

    if(time_step == 0) {
        time_step = 1;
    } else if(time_step > 0x3F)
        time_step = 0x3F;

    program[0] = 0x4000 | val_start; // Set PWM
    if(val_end > val_start) {
        program[1] = (prescaller << 14) | (time_step << 8) | ((diff / 2) & 0x7F); // Ramp Up
    } else {
        program[1] = (prescaller << 14) | (time_step << 8) | 0x80 |
                     ((diff / 2) & 0x7F); // Ramp Down
    }
    program[2] = 0xA001 | ((2 - 1) << 7); // Loop to step 1, repeat twice to get full 8-bit scale
    program[3] = 0xC000; // End

    // Execute program
    result &= lp5562_execute_program(handle, eng, LP5562ChannelWhite, program);

    // Write end value to register
    result &= lp5562_set_channel_value(handle, ch, val_end);
    return result;
}

bool lp5562_execute_blink(
    const FuriHalI2cBusHandle* handle,
    LP5562Engine eng,
    LP5562Channel ch,
    uint16_t on_time,
    uint16_t period,
    uint8_t brightness) {
    bool result = true;

    // Temporary switch to constant value from register
    result &= lp5562_set_channel_src(handle, ch, LP5562Direct);

    // Prepare command sequence
    uint16_t program[16];
    uint16_t time_step = 0;
    uint8_t prescaller = 0;

    program[0] = 0x4000 | brightness; // Set PWM

    time_step = on_time * 2;
    if(time_step > 0x3F) {
        time_step /= 32;
        prescaller = 1;
    } else {
        prescaller = 0;
    }
    if(time_step == 0) {
        time_step = 1;
    } else if(time_step > 0x3F)
        time_step = 0x3F;
    program[1] = (prescaller << 14) | (time_step << 8); // Delay

    program[2] = 0x4000 | 0; // Set PWM

    time_step = (period - on_time) * 2;
    if(time_step > 0x3F) {
        time_step /= 32;
        prescaller = 1;
    } else {
        prescaller = 0;
    }
    if(time_step == 0) {
        time_step = 1;
    } else if(time_step > 0x3F)
        time_step = 0x3F;
    program[3] = (prescaller << 14) | (time_step << 8); // Delay

    program[4] = 0x0000; // Go to start

    // Execute program
    result &= lp5562_execute_program(handle, eng, ch, program);
    return result;
}
