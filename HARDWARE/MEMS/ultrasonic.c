#include "../HARDWARE/include.h"
#include "../HARDWARE/MEMS/ultrasonic.h"
#include "../HARDWARE/DRIVER/usart_fc.h"
#include "../HARDWARE/error.h"
#include "../HARDWARE/MATH/my_math.h"
s8 ultra_start_f;
u8 ultra_ok = 1;
int ultra_distance,ultra_distance_r;
float ultra_delta;

