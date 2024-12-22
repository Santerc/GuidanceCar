//
// Created by 59794 on 2024/12/17.
//

#ifndef PROTO_PC_H
#define PROTO_PC_H


#ifdef __cplusplus
extern "C" {
#endif

    enum Mode{
        kRemoteMode = 0,
        kTrackingMode = 1,
        kMappingMode = 2,
        kGuidanceMode = 3,
    };

    enum Move {
        kStopMove = 0,
        kFrontMove = 1,
        kBackMove = 2,
        kLeftMove = 3,
        kRightMove = 4,
    };

    enum SpeedLevel {
        kLv1 = 1,
        kLv2 = 2,
        kLv3 = 3,
        kLv4 = 4
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
