#ifndef JY901S_H
#define JY901S_H

#include <stdint.h>

typedef struct {
    int16_t roll_raw;
    int16_t pitch_raw;
    int16_t yaw_raw;
    int32_t roll_x10;
    int32_t pitch_x10;
    int32_t yaw_x10;
} JY901S_Angle_t;

void JY901S_Init(void);
void JY901S_ReceiveByte(uint8_t data);
void JY901S_GetAngleRaw(int16_t *roll, int16_t *pitch, int16_t *yaw);
void JY901S_GetAngle(JY901S_Angle_t *angle);
uint8_t JY901S_CalibrateGyroZ(uint16_t sample_count, uint16_t timeout_ms);
void JY901S_ResetGyroZTurnAngle(void);
int32_t JY901S_GetGyroZTurnAngleX10(void);
int32_t JY901S_RawToAngleX10(int16_t raw);
uint32_t JY901S_GetRxCount(void);
uint32_t JY901S_GetGyroFrameCount(void);
uint32_t JY901S_GetAngleFrameCount(void);
uint8_t JY901S_GetLastFrameType(void);

#endif /* JY901S_H */
