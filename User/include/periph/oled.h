      
//
// Created by 18759 on 2024/12/2.
//

#ifndef OLED_H
#define OLED_H
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "i2c_api.h"
#include <vector>
#define OLED_PAGE 8
#define OLED_ROW 8 * OLED_PAGE
#define OLED_COLUMN 128


namespace periph {
    class oled {
    public:
        const uint16_t OLED_DEV_ADDRESS = 0x78;
        const uint16_t OLED_CMD_ADDRESS = 0x00;
        const uint16_t OLED_DATA_ADDRESS = 0x40;
        uint8_t OLED_GRAM[OLED_PAGE][OLED_COLUMN] = {0};

        oled(I2C_HandleTypeDef* i2c): i2c_(i2c) {};
        ~oled() = default;
        void init();
        void clear();
        void setPos(uint8_t seg, uint8_t page);
        void ShowChar8x16(uint8_t seg, uint8_t page, char charactor );
        void ShowChar6x8(uint8_t seg, uint8_t page, char charactor);
        void ShowString8x16(uint8_t seg, uint8_t page, char* string);
        void ShowString6x8(uint8_t seg, uint8_t page, char* string);
        void ShowChinese16x16(uint8_t seg, uint8_t page,uint8_t font_index);
        void send(uint8_t *data, uint8_t len);
        void sendCmd(uint8_t cmd);
        void NewFrame();
        void ShowFrame();

    private:
        void write_cmd(uint8_t cmd);
        void write_data(uint8_t data);



        I2C_HandleTypeDef* i2c_;


    };
}

#endif
#endif //OLED_H

    