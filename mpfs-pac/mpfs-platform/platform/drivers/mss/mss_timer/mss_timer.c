#include "mss_timer.h"

void MSS_TIM1_init(TIMER_TypeDef *timer, mss_timer_mode_t mode)
{
    MSS_TIM1_init_static(timer, mode);
}

void MSS_TIM1_start(TIMER_TypeDef *timer)
{
    MSS_TIM1_start_static(timer);
}

void MSS_TIM1_stop(TIMER_TypeDef *timer)
{
    MSS_TIM1_stop_static(timer);
}

uint32_t MSS_TIM1_get_current_value(TIMER_TypeDef *timer)
{
    return MSS_TIM1_get_current_value_static(timer);
}

void MSS_TIM1_load_immediate(TIMER_TypeDef *timer, uint32_t load_value)
{
    MSS_TIM1_load_immediate_static(timer, load_value);
}

void MSS_TIM1_load_background(TIMER_TypeDef *timer, uint32_t load_value)
{
    MSS_TIM1_load_background_static(timer, load_value);
}

void MSS_TIM1_enable_irq(TIMER_TypeDef *timer)
{
    MSS_TIM1_enable_irq_static(timer);
}

void MSS_TIM1_disable_irq(TIMER_TypeDef *timer)
{
    MSS_TIM1_disable_irq_static(timer);
}

void MSS_TIM1_clear_irq(TIMER_TypeDef *timer)
{
    MSS_TIM1_clear_irq_static(timer);
}

void MSS_TIM2_init(TIMER_TypeDef *timer, mss_timer_mode_t mode)
{
    MSS_TIM2_init_static(timer, mode);
}

void MSS_TIM2_start(TIMER_TypeDef *timer)
{
    MSS_TIM2_start_static(timer);
}

void MSS_TIM2_stop(TIMER_TypeDef *timer)
{
    MSS_TIM2_stop_static(timer);
}

uint32_t MSS_TIM2_get_current_value(TIMER_TypeDef *timer)
{
    return MSS_TIM2_get_current_value_static(timer);
}

void MSS_TIM2_load_immediate(TIMER_TypeDef *timer, uint32_t load_value)
{
    MSS_TIM2_load_immediate_static(timer, load_value);
}

void MSS_TIM2_load_background(TIMER_TypeDef *timer, uint32_t load_value)
{
    MSS_TIM2_load_background_static(timer, load_value);
}

void MSS_TIM2_enable_irq(TIMER_TypeDef *timer)
{
    MSS_TIM2_enable_irq_static(timer);
}

void MSS_TIM2_disable_irq(TIMER_TypeDef *timer)
{
    MSS_TIM2_disable_irq_static(timer);
}

void MSS_TIM2_clear_irq(TIMER_TypeDef *timer)
{
    MSS_TIM2_clear_irq_static(timer);
}

void MSS_TIM64_init(TIMER_TypeDef *timer, mss_timer_mode_t mode)
{
    MSS_TIM64_init_static(timer, mode);
}

void MSS_TIM64_start(TIMER_TypeDef *timer)
{
    MSS_TIM64_start_static(timer);
}

void MSS_TIM64_stop(TIMER_TypeDef *timer)
{
    MSS_TIM64_stop_static(timer);
}

void MSS_TIM64_get_current_value(TIMER_TypeDef *timer, uint32_t *load_value_u, uint32_t *load_value_l)
{
    MSS_TIM64_get_current_value_static(timer, load_value_u, load_value_l);
}

void MSS_TIM64_load_immediate(TIMER_TypeDef *timer, uint32_t load_value_u, uint32_t load_value_l)
{
    MSS_TIM64_load_immediate_static(timer, load_value_u, load_value_l);
}

void MSS_TIM64_load_background(TIMER_TypeDef *timer, uint32_t load_value_u, uint32_t load_value_l)
{
    MSS_TIM64_load_background_static(timer, load_value_u, load_value_l);
}

void MSS_TIM64_enable_irq(TIMER_TypeDef *timer)
{
    MSS_TIM64_enable_irq_static(timer);
}

void MSS_TIM64_enable_irq_for_hart(TIMER_TypeDef *timer, uint64_t hart_id)
{
    MSS_TIM64_enable_irq_for_hart_static(timer, hart_id);
}

void MSS_TIM64_disable_irq(TIMER_TypeDef *timer)
{
    MSS_TIM64_disable_irq_static(timer);
}

void MSS_TIM64_clear_irq(TIMER_TypeDef *timer)
{
    MSS_TIM64_clear_irq_static(timer);
}