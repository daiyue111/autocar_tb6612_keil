#ifndef CAR_APP_H
#define CAR_APP_H

#include <stdint.h>

extern volatile uint8_t gTrackRawMask;
extern volatile uint8_t gTrackBlackMask;
extern volatile uint8_t gTrackActiveCount;
extern volatile int16_t gTrackError;
extern volatile int16_t gTrackDerivative;
extern volatile uint8_t gLeftDuty;
extern volatile uint8_t gRightDuty;
extern volatile uint8_t gStartButtonPressed;
extern volatile uint8_t gLineRunning;
extern volatile uint8_t gMotorTestChannel;
extern volatile uint8_t gImuReady;
extern volatile uint8_t gImuReadOk;
extern volatile uint8_t gImuError;
extern volatile int32_t gGyroZDelta;
extern volatile int32_t gGyroLostHeading;
extern volatile int16_t gGyroCorrectionTicks;
extern volatile int32_t gEncoderLeftCount;
extern volatile int32_t gEncoderRightCount;
extern volatile int16_t gEncoderLeftSpeed;
extern volatile int16_t gEncoderRightSpeed;
extern volatile int16_t gLeftPwmCommand;
extern volatile int16_t gRightPwmCommand;
extern volatile int32_t gHeadingMdeg;
extern volatile uint8_t gMissionState;
extern volatile uint8_t gMissionCornerCount;
extern volatile int32_t gMissionDistanceError;
extern volatile int32_t gMissionHeadingError;
extern volatile uint8_t gPowerSwitchDutyPercent;
extern volatile uint8_t gPowerSwitchOutputOn;
extern volatile uint8_t gPowerSwitchTimedOut;
extern volatile uint8_t gMotionFault;
extern volatile int32_t gEncoderLeftSpeedMmps;
extern volatile int32_t gEncoderRightSpeedMmps;
extern volatile uint8_t gCalibrationStoredValid;
extern volatile uint8_t gCalibrationState;
extern volatile uint8_t gCalibrationFault;
extern volatile uint32_t gCalibrationSequence;
extern volatile uint32_t gCalibrationFlags;
extern volatile int32_t gCalibrationStraightLeftCounts;
extern volatile int32_t gCalibrationStraightRightCounts;
extern volatile uint32_t gCalibrationStraightDistanceMm;
extern volatile uint32_t gCalibrationCountsPerMeter;
extern volatile int32_t gCalibrationTurnLeftCounts;
extern volatile int32_t gCalibrationTurnRightCounts;
extern volatile int32_t gCalibrationTurnHeadingMdeg;
extern volatile uint32_t gCalibrationEffectiveTrackMm;
extern volatile int32_t gCalibrationOffsetLeftCounts;
extern volatile int32_t gCalibrationOffsetRightCounts;
extern volatile uint32_t gCalibrationCornerOffsetMm;

void car_app_init(void);
void car_app_step(void);

#endif
