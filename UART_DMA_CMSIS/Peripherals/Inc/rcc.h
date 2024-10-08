/*
 * rcc.h
 *
 *  Created on: Aug 16, 2024
 *      Author: hlqth
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "main.h"

/*
 * @brief HSE configuration
 */
void rcc_HSE_config(void);

/*
 * @brief SysTick configuration
 * @ param[in] reload value
 */

void rcc_SysTick_config(uint32_t arr);

/*
 * @brief Increment ms Ticks
 */
void rcc_msIncTicks(void);

/*
 * @brief Get ms Ticks
 */
uint32_t rcc_msGetTicks(void);

/*
 * @brief Increment ms Ticks
 */
void rcc_msDelayTicks(uint32_t ms);

#endif /* INC_RCC_H_ */
