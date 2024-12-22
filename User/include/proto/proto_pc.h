//
// Created by 59794 on 2024/12/17.
//

#ifndef PROTO_PC_H
#define PROTO_PC_H


#ifdef __cplusplus
extern "C" {
#endif

    enum Mode{
        kRemoteMode = 1,
        kTrackingMode = 0,
        kMappingMode = 2,
        kGuidanceMode = 3,
    };

    enum Move {
        kFrontMove = 0,
        kBackMove = 1,
        kLeftMove = 2,
        kRightMove = 3,
    };

    enum SpeedLevel {
        kFast = 0,
        kSlow = 1,
    };

    typedef struct{
        uint32_t  timestamp;
        uint8_t connect_;
        uint8_t move_;
        uint8_t mode_;
        uint8_t supercap_;
        uint8_t speed_;
        uint8_t stoptime_;

    }PC2Board_t;

    typedef struct{
        uint32_t  timestamp;
        uint8_t stopFlag_;
        uint8_t move_;

    }Board2PC_t;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus



#endif

#endif //PROTO_PC_H
