#include "mcu_init.h"
#include "io.h"
#include "stm32l4xx.h"
#include "assert_handler.h"
static void init_clocks()
{
    // turn HSI clock on
    RCC->CR |= (0x1 << 8);
    while (!(RCC->CR & (0x1 << 10)))
        ;
    // turn PLL clock off
    RCC->CR &= ~(0x1 << 24);
    while ((RCC->CR & (0x1 << 25)))
        ;
    // setting HSI as PLL source
    RCC->PLLCFGR = (RCC->PLLCFGR & ~0x3) | 0x2;
    // configuring the PLL prescalers to acheive 80Mhz
    RCC->PLLCFGR &= ~(0x7 << 4); // PLLM = 1
    RCC->PLLCFGR = (RCC->PLLCFGR & ~(0x7F << 8)) | (0xA << 8); // PLLN =10
    RCC->PLLCFGR &= ~(0x3 << 25); // PLLR=2
    RCC->PLLCFGR |= (0x1 << 24); // PLLCLCK enabled=7

    /* 4 wait states for flash (configure before switching on PLL)*/
    FLASH->ACR = (FLASH->ACR & ~0x7) | 0x4;
    /* configure the AHB abd APB clock prescalers (configure before switching on PLL)*/
    RCC->CFGR &= ~((0xF << 4) | (0x3 << 8) | (0x3 << 11));

    // turn PLL clock on
    RCC->CR |= (0x1 << 24);
    while (!(RCC->CR & (0x1 << 25)))
        ;

    // turn PLLSAI1 off
    RCC->CR &= ~(0x1 << 26);
    while ((RCC->CR & (0x1 << 27)))
        ;
    /*enable PLLSAI1R as it it used as the ADC clock
     * left at default divider2 */

    RCC->PLLSAI1CFGR &= ~(0x3 << 25); // R
    RCC->PLLSAI1CFGR &= ~(0x3 << 21); // Q
    RCC->PLLSAI1CFGR &= ~(0x1 << 17); // P
    RCC->PLLSAI1CFGR &= ~(0x7F << 8); // N
    RCC->PLLSAI1CFGR |= (0x7 << 8); // N
    RCC->PLLSAI1CFGR |= 0x1 << 24;
    // turn PLLSAI1 on
    RCC->CR |= 0x1 << 26;
    while (!(RCC->CR & (0x1 << 27)))
        ;

    // set sys clock as PLL
    RCC->CFGR &= ~0x3;
    RCC->CFGR = (RCC->CFGR & ~0x3) | 0x3;
    while ((RCC->CFGR & (0x3 << 2)) >> 2 != 0x3)
        ;
}
void mcu_init(void)
{
    init_clocks();
    io_init();
}