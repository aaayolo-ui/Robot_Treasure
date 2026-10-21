/* Emulator only. The fixture writes emulated registers; production readers
 * are bracketed by test_production_call and must never write to MMIO. */
#include "main.h"
#include "Encoder.h"
#include "TreasureMotorSnapshot.h"
#include <stdint.h>

volatile unsigned test_checks, test_failures, test_first_line, test_done;
volatile unsigned test_production_call;
static TreasureMotorSnapshot_t sample;
static TreasureEncoderMap_t mapping;
#define check(x) do { test_checks++; if (!(x)) { test_failures++; \
  if (!test_first_line) test_first_line=__LINE__; } } while (0)

int32_t Encoder_GetTotal(EncoderId_t id)
{ check(id==ENCODER_ID_B); return 222; }

static void probe(uint32_t now,uint16_t count,uint16_t pins)
{
  TIM5->CNT=count; GPIOA->IDR=pins;
  test_production_call=1U;
  TreasureMotorSnapshot_Probe(now);
  test_production_call=0U;
}
static void take(void)
{
  test_production_call=1U;
  TreasureMotorSnapshot_Read(&sample);
  test_production_call=0U;
}

void test_main(void)
{
  unsigned i;
  uint16_t count,pins;
  TIM5->CR1=0x81; TIM5->CCER=0x110;
  TIM5->CCMR1=0x6800; TIM5->CCMR2=0x68;
  TIM5->ARR=999; TIM5->CCR2=750; TIM5->CCR3=400;
  TreasureMotorSnapshot_Init();
  probe(0,64,6); take();
  check(sample.probe_samples==1 && sample.c_checked==0 && sample.d_checked==0);
  for (i=0;i<8;i++) {
    count=(uint16_t)(64U+128U*i);
    pins=(uint16_t)(((count<750U)?2U:0U)|((count<400U)?4U:0U));
    probe(3U+i,count,pins);
  }
  take();
  check(sample.probe_samples==8 && sample.c_checked==8 && sample.d_checked==8);
  check(sample.c_bad==0 && sample.d_bad==0 && sample.phase_mask==0xFFFF);
  check(sample.c_high==6 && sample.d_high==3);
  /* A pin stuck high must disagree in the timer's low phase. */
  probe(20,900,6); take();
  check(sample.c_checked==1 && sample.d_checked==1);
  check(sample.c_bad==1 && sample.d_bad==1);
  /* Edges, CCR preload transitions and unsupported modes are inconclusive. */
  probe(21,746,2); take(); check(sample.c_checked==0);
  TIM5->CCR2=600;
  probe(22,300,6); take(); check(sample.c_checked==0);
  probe(24,300,6); take(); check(sample.c_checked==1 && sample.c_bad==0);
  TIM5->CR1=0; probe(25,300,6); take(); check(sample.c_checked==0 && sample.d_checked==0);
  TIM5->CR1=0x81; TIM5->CCER=0x130;
  probe(26,300,6); take(); check(sample.c_checked==0 && sample.d_checked==1);
  TIM5->CCER=0x110;
  /* Counts saturate together rather than wrapping and producing false ratios. */
  for(i=0;i<65540U;i++) probe(30U+i,300,6);
  take(); check(sample.probe_samples==65535U && sample.c_checked==65535U);
  check(sample.c_high==65535U && sample.c_bad==0);
  take(); check(sample.probe_samples==0 && sample.c_checked==0 && sample.phase_mask==0);
  GPIOC->IDR=GPIO_PIN_2|GPIO_PIN_5; GPIOB->IDR=GPIO_PIN_1;
  take(); check(sample.actual_outputs==0x15);
  TIM3->CNT=111; TIM2->CNT=333; TIM1->CNT=444;
  TIM8->CCR1=10; TIM8->CCR2=20; TIM5->CCR2=30; TIM5->CCR3=40;
  GPIOC->ODR=GPIO_PIN_2;
  test_production_call=1U;
  TreasureMotorSnapshot_ReadEncoderMap(&mapping);
  test_production_call=0U;
  check(mapping.raw_a==111 && mapping.raw_b==222 && mapping.raw_c==333 && mapping.raw_d==444);
  check(mapping.pwm_a==10 && mapping.pwm_b==20 && mapping.pwm_c==30 && mapping.pwm_d==40);
  check(mapping.stby==1);
  check(TIM3->CNT==111 && TIM2->CNT==333 && TIM1->CNT==444);
  test_done=1U;
  for (;;) {}
}
