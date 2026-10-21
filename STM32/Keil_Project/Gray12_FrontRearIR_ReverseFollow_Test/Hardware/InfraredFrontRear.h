#ifndef INFRARED_FRONT_REAR_H
#define INFRARED_FRONT_REAR_H

#include <stdint.h>

/* Both installed obstacle sensors use active-low digital OUT. */
uint8_t InfraredFrontRear_FrontDetected(void);
uint8_t InfraredFrontRear_RearDetected(void);

#endif /* INFRARED_FRONT_REAR_H */
