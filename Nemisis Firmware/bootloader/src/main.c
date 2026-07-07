/*
 * NEMESIS OTA bootloader - B1: jump-only stub.
 *
 * Runs on reset from 0x08000000. For B1 it does nothing but validate the app's
 * vector table and jump to it, to prove the relocation (app based at 0x08008000
 * with VTOR moved) boots and runs correctly through a bootloader handoff.
 *
 * B2 will add, BEFORE the jump: read an "apply" flag from an RTC backup
 * register; if set and the QSPI incoming image is CRC-valid, copy it into the
 * app slot (HAL_FLASH), verify, clear the flag; then jump. SWD/J-Link stays the
 * ultimate recovery net - flashing a normal 0x08000000-based build overwrites
 * both this bootloader and the app slot and boots directly.
 *
 * Deliberately minimal: no clock/peripheral setup. SystemInit() (framework) has
 * already run; the app configures its own clocks, so we leave the MCU in reset
 * defaults and hand over cleanly.
 */
#include "stm32g4xx.h"

#define APP_BASE       0x08008000u
#define RAM_START      0x20000000u
#define RAM_END        0x20020000u          /* 128 KB */

/* Jump to the application at `base`. Never returns on success. */
static void jump_to_app(uint32_t base)
{
  uint32_t app_sp = *(volatile uint32_t *)(base);         /* initial MSP    */
  uint32_t app_pc = *(volatile uint32_t *)(base + 4u);    /* reset vector   */

  /* Sanity-check the app slot: a valid image starts with a stack pointer that
     lives in RAM. A blank slot reads 0xFFFFFFFF, so this rejects "no app". */
  if (app_sp < RAM_START || app_sp > RAM_END)
    return;                                 /* fall through -> caller hangs  */

  __disable_irq();

  /* Undo the little we (and SystemInit) might have left running, so the app
     starts from a clean core state. */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  SCB->VTOR = base;                         /* point the core at the app VT  */
  __DSB();
  __ISB();

  __set_MSP(app_sp);                        /* adopt the app's stack         */

  /* Re-enable interrupts so the app starts exactly as a hardware reset would
     leave it (PRIMASK=0). Without this the app inherits our __disable_irq(),
     SysTick never fires, uwTick never advances, and every HAL_Delay() hangs
     forever - the app runs but appears dead. */
  __enable_irq();

  ((void (*)(void))app_pc)();               /* branch to the app reset handler */
}

int main(void)
{
  jump_to_app(APP_BASE);

  /* Only reached if the app slot is invalid/blank. Hang so J-Link can attach
     and recover rather than looping into random flash. (B3 will fall back to
     the golden image here instead of hanging.) */
  while (1)
  {
    __NOP();
  }
}
