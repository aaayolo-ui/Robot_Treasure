#include "AI1811Mux.h"
#include <string.h>
#define MODE_UNKNOWN 0U
#define MODE_RAW 1U
#define MODE_HEX 2U

void AI1811Mux_ResetStream(AI1811Mux *m)
{
  m->channel=AI1811_MUX_CHANNELS; m->prefix=0U; m->mode=MODE_UNKNOWN;
  m->count=m->high=m->has_high=0U;
}
void AI1811Mux_Init(AI1811Mux *m)
{
  memset(m,0,sizeof(*m)); AI1811Mux_ResetStream(m);
}
static int Hex(uint8_t b)
{
  if(b>='0' && b<='9') return b-'0';
  if(b>='A' && b<='F') return b-'A'+10;
  if(b>='a' && b<='f') return b-'a'+10;
  return -1;
}
static void FrameByte(AI1811Mux *m,uint8_t b,uint32_t tick,
                      AI1811Mux_Handler handler,void *context)
{
  uint8_t sum=0U,i;
  if(!m->count) {
    if(b==0xAAU) { m->frame[0]=b; m->count=1U; m->first_tick=tick; }
    return;
  }
  if(m->count==1U && b!=0x05U) {
    m->count=0U;
    if(b==0xAAU) {m->frame[0]=b;m->count=1U;m->first_tick=tick;}
    return;
  }
  m->frame[m->count++]=b;
  if(m->count<8U) return;
  for(i=0U;i<7U;i++) sum=(uint8_t)(sum+m->frame[i]);
  if(sum!=m->frame[7] || (uint32_t)(tick-m->first_tick)>AI1811_MUX_FRESH_MS) {
    m->bad[m->channel]++;
    /* A damaged multiplex packet must reacquire its channel prefix. */
    AI1811Mux_ResetStream(m); return;
  }
  m->valid[m->channel]++;
  if(handler) handler(context,m->channel,
      (uint16_t)(m->frame[4]|((uint16_t)m->frame[5]<<8)),m->frame[6],
      (uint16_t)(m->frame[2]|((uint16_t)m->frame[3]<<8)),tick);
  m->count=0U;
}
void AI1811Mux_Byte(AI1811Mux *m,uint8_t b,uint32_t tick,
                    AI1811Mux_Handler handler,void *context)
{
  int digit;
  if((uint32_t)(tick-m->last_tick)>AI1811_MUX_FRESH_MS) {
    if(m->count && m->channel<AI1811_MUX_CHANNELS) m->bad[m->channel]++;
    AI1811Mux_ResetStream(m);
  }
  m->last_tick=tick;
  /* Prefixes are recognized only between frames. A literal '1#' inside an
   * eight-byte binary payload must never change its physical direction. */
  if(!m->count) {
    if(m->prefix) {
      if(b=='#') {
        m->channel=(uint8_t)(m->prefix-'1'); m->prefix=0U;
        m->mode=MODE_UNKNOWN; m->has_high=0U; return;
      }
      m->prefix=0U;
    }
    if(b>='1' && b<='4') {m->prefix=b;return;}
  }
  if(m->channel>=AI1811_MUX_CHANNELS) return;
  if(m->mode==MODE_UNKNOWN) {
    if(b==0xAAU) m->mode=MODE_RAW;
    else if(b=='A' || b=='a') m->mode=MODE_HEX;
    else return;
  }
  if(m->mode==MODE_RAW) {FrameByte(m,b,tick,handler,context);return;}
  digit=Hex(b);
  if(digit<0) {
    if((b==' ' || b=='\t' || b=='\r' || b=='\n') && !m->has_high) return;
    m->bad[m->channel]++; AI1811Mux_ResetStream(m); return;
  }
  if(!m->has_high) {m->high=(uint8_t)digit;m->has_high=1U;return;}
  b=(uint8_t)((m->high<<4)|(uint8_t)digit);m->has_high=0U;
  FrameByte(m,b,tick,handler,context);
}
