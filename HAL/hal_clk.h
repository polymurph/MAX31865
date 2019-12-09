/*
 * hal_clk.h
 *
 *  Created on: Nov 21, 2019
 *      Author: Edwin Koch
 */

#ifndef HAL_HAL_CLK_H_
#define HAL_HAL_CLK_H_

#include <stdint.h>
#include <stdbool.h>

#include "HAL/msp430fr6989.h"

typedef enum{
    clk_error_NO_ERROR = 0,
    clk_error_LFXT_FAULT,
    clk_error_HFXT_FAULT

}clk_error_t;

typedef enum{
    clk_ACLK_src_LFXT,
    clk_ACLK_src_VLO,
    clk_ACLK_src_LFMOD
}clk_ACLK_src_t;

typedef enum{
    clk_MCLK_src_LFXT,
    clk_MCLK_src_VLO,
    clk_MCLK_src_LFMOD,
    clk_MCLK_src_DCO,
    clk_MCLK_src_MOD,
    clk_MCLK_src_HFXT
}clk_MCLK_src_t;

typedef enum{
    clk_SMCLK_src_LFXT,
    clk_SMCLK_src_VLO,
    clk_SMCLK_src_LFMOD,
    clk_SMCLK_src_DCO,
    clk_SMCLK_src_MOD,
    clk_SMCLK_src_HFXT
}clk_SMCLK_src_t;

// TODO: find problem with 16/21/24 MHz (all not working)
typedef enum{
    clk_dco_freq_1_MHz = 0,
    clk_dco_freq_2_67_MHz = DCOFSEL0,
    clk_dco_freq_3_5_MHz = DCOFSEL1,
    clk_dco_freq_4_MHz = DCOFSEL1 | DCOFSEL0,
    clk_dco_freq_5_33_MHz = DCOFSEL2,
    clk_dco_freq_7_MHz = DCOFSEL2 | DCOFSEL0,
    clk_dco_freq_8_MHz = DCOFSEL2 | DCOFSEL1,
    clk_dco_freq_16_MHz = DCOFSEL2 | DCORSEL,
    clk_dco_freq_21_MHz = DCOFSEL2 | DCOFSEL0 | DCORSEL,
    clk_dco_freq_24_MHz = DCOFSEL2 | DCOFSEL1 | DCORSEL
}clk_dco_freq_t;

typedef enum{
    clk_presc_DIV_1 = 0,
    clk_presc_DIV_2,
    clk_presc_DIV_4,
    clk_presc_DIV_8,
    clk_presc_DIV_16,
    clk_presc_DIV_32
}clk_presc_t;

typedef enum{
    drive_strenght_LOW = 0,
    drive_strenght_INCREASED = 1,
    drive_strenght_MAX = 3
}drive_strenght_t;

clk_error_t hal_clk_config_LFXT(drive_strenght_t    strength,
                                bool                bypass,
                                bool                enable);

clk_error_t hal_clk_config_HFXT(drive_strenght_t    strength,
                                bool                bypass,
                                bool                enable);

void hal_clk_config_DCO(clk_dco_freq_t freq);

void hal_clk_config_ACLK(clk_ACLK_src_t     source,
                         clk_presc_t        prescaler,
                         bool               enable);

void hal_clk_config_MCLK(clk_MCLK_src_t     source,
                         clk_presc_t        prescaler,
                         bool               request_enable);

void hal_clk_config_SMCLK(clk_SMCLK_src_t   source,
                          clk_presc_t       prescale,
                          bool              request_enable,
                          bool              enable);

void hal_clk_output_MCLK_to_GPIO(bool enable);

void hal_clk_output_ACLK_to_GPIO(bool enable);

#endif /* HAL_HAL_CLK_H_ */
