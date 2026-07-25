# STM32 Project Repo

This is my personal STM32 Project Repo. Projects are mostly done in a `Nucleo-F446ZE` development board

## Content

### 1. PWM Method Comparison

Compare the difference between Sine PWM, Geometric SVPWM, and Third Harmonic SVPWM and find the better method accuracy wise and cost wise.

### 2. PWM Test

Learning how to enable pins in STM32 F4

## STM32F4 F446ZE Pinout

| No  |         Function          |   GPIO    |  Pin  | Board Pin | Comment |
| --- | :-----------------------: | :-------: | :---: | :-------: | :------ |
| 1   |          PWM A+           | TIM1_CH1  |  PE9  |    xx     |         |
| 2   |          PWM A-           | TIM1_CH1N |  PA7  |    xx     |         |
| 3   |          PWM B+           | TIM1_CH2  | PE11  |           |         |
| 4   |          PWM B-           | TIM1_CH2N |  PB0  |           |         |
| 5   |          PWM C+           | TIM1_CH3  | PE13  |           |         |
| 6   |          PWM C-           | TIM1_CH3N |  PB1  |           |         |
| 7   |  Calculate PWM Interrupt  |  GPIOE_8  |  PG8  |           |         |
| 8   | Calculate PWM Interrupt 2 |  GPIOG_8  |  PE8  |           |         |

