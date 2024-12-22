//
// Created by 18759 on 2024/12/2.
//

#ifndef BOSON_H
#define BOSON_H
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "gpio_api.h"



namespace periph {
    class boson {
    public:
        boson(bsp::GPIO gpio) : gpio_(gpio) {
            for (int i = 4; i >= 0; i--) {
                result_[i] = false;
            }
        }
        boson();
        ~boson() = default;
        void Detect() {
            for (int i = 4; i > 0; i--) {
                result_[i] = result_[i-1];
            }
            result_[0] = (!gpio_.ReadPin());
        };
        [[nodiscard]] bool GetColor() const {
            float sum = 0;
            for (int i = 0; i < 5; i ++) {
                sum += result_[i];
            }
            return (sum > 0);
        }
    private:
        bsp::GPIO gpio_;
        bool result_[5];
    };
}

#endif
#endif //BOSON_H
