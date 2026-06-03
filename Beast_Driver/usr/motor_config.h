#ifndef MOTOR_CONFIG_H_
#define MOTOR_CONFIG_H_

// *******************************************************************************************************
//                                         Motor Config
// *******************************************************************************************************
// 1. select CAN_ID
#define CAN_ID 0x03 // Can ID

// 2. Choose Motor Type
#define MOTOR_6210 // Motor Type:{MOTOR_8110, MOTOR_8115, MOTOR_10015}

// 3. Dual_Encoder Type
#ifdef ROTOR_SPI2
#define DUAL_ENCODER_SPI // Encoder Type:{DUAL_ENCODER_SPI, DUAL_ENCODER_USART, DUAL_ENCODER_NULL}
#else
#define DUAL_ENCODER_NULL
#endif

#ifdef MOTOR_8110
#define L_s 0.0000435f    // phase resistor
#define R_s 0.0619215f    // phase resistance
#define Npp 21            // pole pairs
#define phi_m 0.00391969f // flux linkage
#define K_t 0.12347f
// #define Inverse_KT_Out 0.83068035f // 1/(K_t * GR)
#define Inverse_KT_Out 2.8863112f
#define KT_Out 0.346463f
#define Damping_Fac 0.00000303448f // SI unit
#define Inertia 0.000095332f       // SI unit
#define K_a 0.213213f              // current loop
#define K_b 1423.5f                // current loop
#define K_av 0.1415f               // vel loop
#define K_bv 13.3585f              // vel loop
#define Kp_position 40.f           // position loop, no more than 50
#define GR 9.75f
#define Inverse_GR 0.1025641f // 1/GR
#endif

#ifdef MOTOR_8115
#define L_s 0.000105f     // phase resistor
#define R_s 0.1625f       // phase resistance
#define Npp 21            // pole pairs
#define phi_m 0.0043830630695346f // flux linkage
#define K_t 0.1380664866903399f
// #define Inverse_KT_Out 0.5450261054f // 1/(K_t * GR)
#define Inverse_KT_Out 0.7428602336650801f // 1/(K_t * GR)
#define KT_Out 1.346148245230814f
#define Damping_Fac 0.00000303448f // SI unit
#define Inertia 0.000278265f       // SI unit
#define K_a 0.05f                  // current loop  // nearly 10000hz
#define K_b 1547.6190476190f       // current loop
#define K_av 0.6013f               // vel loop
#define K_bv 13.5926f              // vel loop
#define Kp_position 40.f           // position loop, no more than 50
#define GR 9.75f
#define Inverse_GR 0.1025641f // 1/GR
#define di_MAX 13.5f // 10NM
#endif

#ifdef MOTOR_6210
#define L_s 0.000108832f     // phase resistor
#define R_s 0.1423715f       // phase resistance
#define Npp 14            // pole pairs
#define phi_m 0.005257336f // flux linkage
#define K_t 0.0637418f
// #define Inverse_KT_Out 0.5450261054f // 1/(K_t * GR)
#define Inverse_KT_Out 1.0646247f // 1/(K_t * GR)
#define KT_Out 0.939298115f
#define Damping_Fac 0.00000303448f // SI unit
#define Inertia 0.000278265f       // SI unit
#define K_a 0.15f                  // current loop  // nearly 10000hz
#define K_b 1308.1768f       // current loop
#define K_av 0.6013f               // vel loop
#define K_bv 13.5926f              // vel loop
#define Kp_position 40.f           // position loop, no more than 50
#define GR 10.75f
#define Inverse_GR 0.093023256f // 1/GR
#define di_MAX 9.4f // 10NM
#endif
#endif