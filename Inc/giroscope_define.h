#ifndef L3GD20_GYRO_H
#define L3GD20_GYRO_H

//  L3G4200DMEMS Address
//  7bit address = 0b110100x(0x68 or 0x69 depends on SA0/SDO)
#define L3G4200D_G_CHIP_ADDR (0x68 << 1)    // SA0(=SDO pin) = Ground
#define L3G4200D_V_CHIP_ADDR (0x69 << 1)    // SA0(=SDO pin) = Vdd
//  L3GD20MEMS Address
//  7bit address = 0b110101x(0x6a or 0x6b depends on SA0/SDO)
#define L3GD20_G_CHIP_ADDR   (0x6a << 1)    // SA0(=SDO pin) = Ground
#define L3GD20_V_CHIP_ADDR   (0x6b << 1)    // SA0(=SDO pin) = Vdd

//  L3G4200DMEMS ID
#define I_AM_L3G4200D        0xd3
//  L3GD20MEMS ID
#define I_AM_L3GD20          0xd4

//  Register's definition
#define L3GX_WHO_AM_I        0x0f
#define L3GX_CTRL_REG1       0x20
#define L3GX_CTRL_REG2       0x21
#define L3GX_CTRL_REG3       0x22
#define L3GX_CTRL_REG4       0x23
#define L3GX_CTRL_REG5       0x24
#define L3GX_REFERENCE       0x25
#define L3GX_OUT_TEMP        0x26
#define L3GX_STATUS_REG      0x27
#define L3GX_OUT_X_L         0x28
#define L3GX_OUT_X_H         0x29
#define L3GX_OUT_Y_L         0x2a
#define L3GX_OUT_Y_H         0x2b
#define L3GX_OUT_Z_L         0x2c
#define L3GX_OUT_Z_H         0x2d
#define L3GX_FIFO_CTRL_REG   0x2e
#define L3GX_FIFO_SRC_REG    0x2f
#define L3GX_INT1_CFG        0x30
#define L3GX_INT1_SRC        0x31
#define L3GX_INT1_TSH_XH     0x32
#define L3GX_INT1_TSH_XL     0x33
#define L3GX_INT1_TSH_YH     0x34
#define L3GX_INT1_TSH_YL     0x35
#define L3GX_INT1_TSH_ZH     0x36
#define L3GX_INT1_TSH_ZL     0x37
#define L3GX_INT1_DURATION   0x38

// Output Data Rate (ODR)
//      L3G4200DMEMS
#define L3GX_DR_100HZ        0
#define L3GX_DR_200HZ        1
#define L3GX_DR_400HZ        2
#define L3GX_DR_800HZ        3
//      L3GD20MEMS
#define L3GX_DR_95HZ         0
#define L3GX_DR_190HZ        1
#define L3GX_DR_380HZ        2
#define L3GX_DR_760HZ        3

// Bandwidth (Low pass)
#define L3GX_BW_LOW          0
#define L3GX_BW_M_LOW        1
#define L3GX_BW_M_HI         2
#define L3GX_BW_HI           3

// Power-down mode enable/disable
#define L3GX_PD_EN           0
#define L3GX_PD_DIS          1

// Axis control
#define L3GX_X_EN            1
#define L3GX_X_DIS           0
#define L3GX_Y_EN            1
#define L3GX_Y_DIS           0
#define L3GX_Z_EN            1
#define L3GX_Z_DIS           0

// Full Scale
#define L3GX_FS_250DPS       0
#define L3GX_FS_500DPS       1
#define L3GX_FS_2000DPS      2

#endif
