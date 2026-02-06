#include "bpu_r4.h"
#include <stdarg.h>

// -----------------------------------------------------------------------------
// Streams
// -----------------------------------------------------------------------------
static HardwareSerial& LOG = Serial;
static HardwareSerial& OUT = Serial1;

// -----------------------------------------------------------------------------
// Pins / Baud
// -----------------------------------------------------------------------------
static const uint32_t LOG_BAUD = 115200;
static const uint32_t OUT_BAUD = 921600;
static const int OUT_TX_PIN = 17;
static const int OUT_RX_PIN = 16;

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------
static const uint32_t TICK_MS   = 20;
static const uint32_t SENSOR_MS = 5;
static const uint32_t HB_MS     = 50;
static const uint32_t TELEM_MS  = 200;

// -----------------------------------------------------------------------------
// Queue / budget
// -----------------------------------------------------------------------------
static const uint16_t TX_BUDGET_BYTES = 800;
static const int OUT_MIN_FREE = 96;

static const bool ENABLE_DEGRADE = true;
static const bool DEBUG_DUMP_TX_HEX = false;

// -----------------------------------------------------------------------------
// Coalesce / aged
// -----------------------------------------------------------------------------
static const uint32_t COALESCE_WINDOW_MS = 20;
static const uint32_t AGED_MS = 200;

// -----------------------------------------------------------------------------
// Stress draw
// -----------------------------------------------------------------------------
static const bool  ENABLE_DRAW_STRESS = true;
static const uint16_t DRAW_PER_TICK = 40;
static const uint8_t DRAW_POINT_BYTES = 5;
static const uint8_t DRAW_PAYLOAD_MAX = 62;
static const uint8_t DRAW_MAXPTS_PER_JOB = (DRAW_PAYLOAD_MAX - 2) / DRAW_POINT_BYTES;

// -----------------------------------------------------------------------------
// Queues size
// -----------------------------------------------------------------------------
static const size_t EVT_QN      = 256;
static const size_t DRAWQ_N     = 128;
static const size_t CTRLQ_N     = 64;

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------
enum : uint8_t { EVT_CMD=1, EVT_SENSOR=2, EVT_HB=3, EVT_TELEM=4, EVT_DRAW=5 };
enum : uint8_t { JOB_CMD=1, JOB_SENSOR=2, JOB_HB=3, JOB_TELEM=4, JOB_DRAW=5 };
enum MergePolicy : uint8_t { MERGE_NONE=0, MERGE_LAST=1 };

static inline MergePolicy policy_for_evt(uint8_t type){
  switch(type){
    case EVT_SENSOR: return MERGE_LAST;
    case EVT_HB:     return MERGE_LAST;
    case EVT_TELEM:  return MERGE_LAST;
    default:         return MERGE_NONE;
  }
}
static inline uint8_t job_for_evt(uint8_t evt_type){
  switch(evt_type){
    case EVT_CMD:    return JOB_CMD;
    case EVT_SENSOR: return JOB_SENSOR;
    case EVT_HB:     return JOB_HB;
    case EVT_TELEM:  return JOB_TELEM;
    case EVT_DRAW:   return JOB_DRAW;
    default:         return 0;
  }
}
static inline bool ctrl_merge_allowed(uint8_t job_type){
  return (job_type == JOB_SENSOR || job_type == JOB_HB || job_type == JOB_TELEM);
}
static inline uint64_t bit64(uint8_t n){ return (n>=64)?0ULL:(1ULL<<n); }

// -----------------------------------------------------------------------------
// Data structs
// -----------------------------------------------------------------------------
struct BpuEvent {
  uint8_t  type;
  uint8_t  flags;
  uint16_t len;
  uint32_t t_ms;
  uint8_t  payload[16];
};
struct BpuJob {
  uint8_t  type;
  uint8_t  flags;
  uint16_t len;
  uint32_t t_ms;
  uint8_t  payload[64];
};
struct BpuStats {
  uint32_t tick=0;
  uint32_t ev_in=0, ev_out=0, ev_merge=0, ev_drop=0;

  uint32_t job_in=0;
  uint32_t job_out_draw=0, job_out_ctrl=0;
  uint32_t job_drop_draw=0, job_drop_ctrl=0;
  uint32_t mergeCtrl=0;

  uint32_t uart_sent=0, uart_skipB=0, uart_skipTX=0, uart_bytes=0;
  uint32_t flush_try=0, flush_ok=0, flush_drawOk=0, flush_ctrlOk=0;
  uint32_t flush_partial=0, flush_full=0;

  uint32_t pick_s=0, pick_h=0, pick_t=0, pick_d=0, pick_aged=0;

  uint32_t degrade_drop=0, degrade_requeue=0, oldDrawDrop=0;

  uint32_t draw_ev=0, draw_jobs=0, draw_pts=0;

  uint32_t work_us_last=0, work_us_max=0;

  uint32_t out_bytes_total=0;
  uint32_t log_bytes_total=0;
};

// -----------------------------------------------------------------------------
// Ring buffer
// -----------------------------------------------------------------------------
template<typename T, size_t N>
struct Ring {
  T buf[N];
  uint16_t head=0, tail=0, count=0;

  bool push(const T& v){
    if(count>=N) return false;
    buf[head]=v;
    head=(head+1)%N;
    count++;
    return true;
  }
  bool pop(T& out){
    if(count==0) return false;
    out=buf[tail];
    tail=(tail+1)%N;
    count--;
    return true;
  }
  bool drop_oldest(){
    if(count==0) return false;
    tail=(tail+1)%N;
    count--;
    return true;
  }
  T& at(size_t idx){ return buf[(tail+idx)%N]; }
};

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
static BpuStats st;
static Ring<BpuEvent, EVT_QN> evq;
static Ring<BpuJob, DRAWQ_N>  drawq;
static Ring<BpuJob, CTRLQ_N>  ctrlq;

static uint8_t  g_seq = 0;
static uint32_t t_next_sensor=0, t_next_hb=0, t_next_telem=0;

static uint32_t g_rng=0xC0FFEE12u;
static inline uint32_t lcg(){ g_rng = g_rng*1664525u + 1013904223u; return g_rng; }

// -----------------------------------------------------------------------------
// Logging
// -----------------------------------------------------------------------------
static void logf(const char* fmt, ...){
  char buf[512];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if(n<=0) return;
  if(n>=(int)sizeof(buf)) n=(int)sizeof(buf)-1;
  LOG.print(buf);
  st.log_bytes_total += (uint32_t)n;
}

// -----------------------------------------------------------------------------
// CRC16
// -----------------------------------------------------------------------------
static uint16_t crc16_ccitt(const uint8_t* data, size_t len){
  uint16_t crc=0xFFFF;
  for(size_t i=0;i<len;i++){
    crc ^= (uint16_t)data[i] << 8;
    for(int b=0;b<8;b++){
      if(crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else            crc = (crc << 1);
    }
  }
  return crc;
}

// -----------------------------------------------------------------------------
// COBS encode
// -----------------------------------------------------------------------------
static size_t cobs_encode(const uint8_t* input, size_t length, uint8_t* output, size_t out_max){
  if(out_max==0) return 0;
  size_t read_index=0, write_index=1, code_index=0;
  uint8_t code=1;

  while(read_index < length){
    if(write_index >= out_max) return 0;
    if(input[read_index] == 0){
      output[code_index] = code;
      code=1;
      code_index = write_index++;
      read_index++;
    } else {
      output[write_index++] = input[read_index++];
      code++;
      if(code == 0xFF){
        output[code_index] = code;
        code=1;
        code_index = write_index++;
      }
    }
  }
  if(code_index >= out_max) return 0;
  output[code_index] = code;
  return write_index;
}

// -----------------------------------------------------------------------------
// UART frame send
// -----------------------------------------------------------------------------
static bool uart_send_frame(uint8_t type, const uint8_t* payload, uint8_t len, uint16_t& bytes_sent_out){
  if(len > 64) return false;

  uint8_t decoded[4 + 64 + 2];
  decoded[0]=0xB2;
  decoded[1]=type;
  decoded[2]=g_seq++;
  decoded[3]=len;
  for(uint8_t i=0;i<len;i++) decoded[4+i]=payload[i];

  uint16_t crc = crc16_ccitt(&decoded[1], (size_t)(3+len));
  decoded[4+len+0]=(uint8_t)(crc & 0xFF);
  decoded[4+len+1]=(uint8_t)((crc >> 8) & 0xFF);

  const size_t decoded_len = (size_t)(4 + len + 2);

  uint8_t encoded[4 + 64 + 2 + 16];
  size_t enc_len = cobs_encode(decoded, decoded_len, encoded, sizeof(encoded));
  if(enc_len == 0) return false;

  bytes_sent_out = (uint16_t)(enc_len + 1);

  OUT.write(encoded, enc_len);
  OUT.write((uint8_t)0x00);

  st.out_bytes_total += bytes_sent_out;

  return true;
}

// -----------------------------------------------------------------------------
// Event queue
// -----------------------------------------------------------------------------
static bool evq_push(const BpuEvent& e){
  st.ev_in++;

  if(COALESCE_WINDOW_MS>0 && policy_for_evt(e.type)==MERGE_LAST && evq.count>0){
    for(size_t i=0;i<evq.count;i++){
      BpuEvent& ex = evq.at(i);
      if(ex.type==e.type){
        if((uint32_t)(e.t_ms - ex.t_ms) <= COALESCE_WINDOW_MS){
          ex = e;
          st.ev_merge++;
          return true;
        }
      }
    }
  }

  if(!evq.push(e)){
    st.ev_drop++;
    return false;
  }
  return true;
}

static bool evq_pop_evt(BpuEvent& out){
  if(!evq.pop(out)) return false;
  st.ev_out++;
  return true;
}

// -----------------------------------------------------------------------------
// CTRL queue
// -----------------------------------------------------------------------------
static bool ctrlq_push_merge_last(const BpuJob& j){
  st.job_in++;

  if(ctrl_merge_allowed(j.type)){
    for(size_t i=0;i<ctrlq.count;i++){
      BpuJob& ex = ctrlq.at(i);
      if(ex.type == j.type){
        ex = j;
        st.mergeCtrl++;
        return true;
      }
    }
  }

  if(!ctrlq.push(j)){
    st.job_drop_ctrl++;
    return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// DRAW queue
// -----------------------------------------------------------------------------
static bool drawq_push(const BpuJob& j){
  st.job_in++;
  if(!drawq.push(j)){
    st.job_drop_draw++;
    return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// Dirty derived
// -----------------------------------------------------------------------------
static uint64_t dirty_derived(){
  uint64_t m=0;
  for(size_t i=0;i<drawq.count;i++){
    const BpuJob& j = drawq.at(i);
    if(j.type>=1 && j.type<=63) m |= bit64(j.type);
  }
  for(size_t i=0;i<ctrlq.count;i++){
    const BpuJob& j = ctrlq.at(i);
    if(j.type>=1 && j.type<=63) m |= bit64(j.type);
  }
  return m;
}

// -----------------------------------------------------------------------------
// Sources
// -----------------------------------------------------------------------------
static void schedule_sources(uint32_t now_ms){
  if((int32_t)(now_ms - t_next_sensor) >= 0){
    t_next_sensor = now_ms + SENSOR_MS;
    BpuEvent e{};
    e.type=EVT_SENSOR; e.t_ms=now_ms; e.len=2;
    uint16_t v=(uint16_t)((now_ms/10)&0xFFFF);
    e.payload[0]=(uint8_t)(v&0xFF);
    e.payload[1]=(uint8_t)((v>>8)&0xFF);
    st.pick_s++;
    evq_push(e);
  }

  if((int32_t)(now_ms - t_next_hb) >= 0){
    t_next_hb = now_ms + HB_MS;
    BpuEvent e{};
    e.type=EVT_HB; e.t_ms=now_ms; e.len=1;
    e.payload[0]=0x01;
    st.pick_h++;
    evq_push(e);
  }

  if((int32_t)(now_ms - t_next_telem) >= 0){
    t_next_telem = now_ms + TELEM_MS;
    BpuEvent e{};
    e.type=EVT_TELEM; e.t_ms=now_ms; e.len=4;
    uint32_t m=now_ms;
    e.payload[0]=(uint8_t)(m&0xFF);
    e.payload[1]=(uint8_t)((m>>8)&0xFF);
    e.payload[2]=(uint8_t)((m>>16)&0xFF);
    e.payload[3]=(uint8_t)((m>>24)&0xFF);
    st.pick_t++;
    evq_push(e);
  }

  if(ENABLE_DRAW_STRESS){
    for(uint16_t i=0;i<DRAW_PER_TICK;i++){
      BpuEvent e{};
      e.type=EVT_DRAW; e.t_ms=now_ms; e.len=5;

      uint32_t r=lcg();
      uint16_t x=(uint16_t)(r & 0x03FF);
      uint16_t y=(uint16_t)((r>>10) & 0x03FF);
      uint8_t  p=(uint8_t)((r>>20) & 0xFF);

      e.payload[0]=(uint8_t)(x&0xFF);
      e.payload[1]=(uint8_t)((x>>8)&0xFF);
      e.payload[2]=(uint8_t)(y&0xFF);
      e.payload[3]=(uint8_t)((y>>8)&0xFF);
      e.payload[4]=p;

      st.pick_d++;
      st.draw_ev++;
      evq_push(e);
    }
  }
}

// -----------------------------------------------------------------------------
// Events -> Jobs
// -----------------------------------------------------------------------------
static void push_draw_job(uint32_t now_ms, const uint8_t* buf, uint8_t cnt){
  if(cnt==0) return;

  BpuJob j{};
  j.type=JOB_DRAW;
  j.t_ms=now_ms;

  j.payload[0]=0x10;
  j.payload[1]=cnt;

  uint16_t dlen = (uint16_t)cnt * DRAW_POINT_BYTES;
  for(uint16_t i=0;i<dlen;i++) j.payload[2+i]=buf[i];
  j.len = (uint16_t)(2 + dlen);

  if(drawq_push(j)){
    st.draw_jobs++;
    st.draw_pts += cnt;
  }
}

static void schedule_from_events(uint32_t now_ms){
  uint8_t draw_buf[DRAW_PAYLOAD_MAX];
  uint8_t draw_cnt=0;

  BpuEvent e{};
  while(evq_pop_evt(e)){
    if(e.type==EVT_DRAW){
      if(draw_cnt >= DRAW_MAXPTS_PER_JOB){
        push_draw_job(now_ms, draw_buf, draw_cnt);
        draw_cnt=0;
      }

      uint16_t x=(uint16_t)e.payload[0] | ((uint16_t)e.payload[1]<<8);
      uint16_t y=(uint16_t)e.payload[2] | ((uint16_t)e.payload[3]<<8);
      uint8_t  p=e.payload[4];

      uint16_t off=(uint16_t)draw_cnt * DRAW_POINT_BYTES;
      draw_buf[off+0]=(uint8_t)(x&0xFF);
      draw_buf[off+1]=(uint8_t)((x>>8)&0xFF);
      draw_buf[off+2]=(uint8_t)(y&0xFF);
      draw_buf[off+3]=(uint8_t)((y>>8)&0xFF);
      draw_buf[off+4]=p;
      draw_cnt++;
      continue;
    }

    if(draw_cnt>0){
      push_draw_job(now_ms, draw_buf, draw_cnt);
      draw_cnt=0;
    }

    BpuJob j{};
    j.type = job_for_evt(e.type);
    j.t_ms = now_ms;

    uint8_t tag=0;
    if(e.type==EVT_SENSOR) tag=0x01;
    if(e.type==EVT_HB)     tag=0x02;
    if(e.type==EVT_TELEM)  tag=0x03;
    if(e.type==EVT_CMD)    tag=0x04;

    j.payload[0]=tag;
    j.payload[1]=(uint8_t)e.len;

    uint16_t copy_n=e.len;
    if(copy_n > (sizeof(j.payload)-2)) copy_n=(uint16_t)(sizeof(j.payload)-2);
    for(uint16_t i=0;i<copy_n;i++) j.payload[2+i]=e.payload[i];
    j.len=(uint16_t)(2+copy_n);

    ctrlq_push_merge_last(j);
  }

  if(draw_cnt>0){
    push_draw_job(now_ms, draw_buf, draw_cnt);
  }
}

// -----------------------------------------------------------------------------
// Flush job
// -----------------------------------------------------------------------------
static bool flush_job(const BpuJob& j, uint16_t& budget_left, uint16_t& sent_bytes_out){
  sent_bytes_out = 0;

  size_t decoded_len = 4 + j.len + 2;
  size_t worst_overhead = (decoded_len / 254) + 2;
  size_t worst_on_wire  = decoded_len + worst_overhead + 1;

  if(worst_on_wire > budget_left){
    st.uart_skipB++;
    return false;
  }
  if(OUT.availableForWrite() < OUT_MIN_FREE){
    st.uart_skipTX++;
    return false;
  }
  if(!uart_send_frame(j.type, j.payload, (uint8_t)j.len, sent_bytes_out)){
    return false;
  }

  budget_left = (uint16_t)(budget_left - sent_bytes_out);
  st.uart_sent++;
  st.uart_bytes += sent_bytes_out;
  st.flush_ok++;
  return true;
}

// -----------------------------------------------------------------------------
// Tick
// -----------------------------------------------------------------------------
static void bpu_tick(uint32_t now_ms){
  uint32_t t0=(uint32_t)micros();

  schedule_sources(now_ms);
  schedule_from_events(now_ms);

  uint16_t budget = TX_BUDGET_BYTES;
  bool sent_any=false;

  // draw first
  while(budget > 0 && drawq.count > 0){
    st.flush_try++;
    BpuJob j{};
    if(!drawq.pop(j)) break;

    uint16_t sent=0;
    if(!flush_job(j, budget, sent)){
      st.degrade_drop++;
      st.oldDrawDrop++;
      break;
    }
    sent_any=true;
    st.flush_drawOk++;
    st.job_out_draw++;
  }

  // ctrl
  while(budget > 0 && ctrlq.count > 0){
    st.flush_try++;
    BpuJob j{};
    if(!ctrlq.pop(j)) break;

    uint16_t sent=0;
    if(!flush_job(j, budget, sent)){
      if(ENABLE_DEGRADE && j.type==JOB_TELEM){
        st.degrade_drop++;
      } else {
        ctrlq_push_merge_last(j);
        st.degrade_requeue++;
      }
      break;
    }
    sent_any=true;
    st.flush_ctrlOk++;
    st.job_out_ctrl++;
  }

  if(sent_any){
    if(drawq.count==0 && ctrlq.count==0) st.flush_full++;
    else                                st.flush_partial++;
  }

  st.tick++;

  uint32_t t1=(uint32_t)micros();
  uint32_t work_us=(t1>=t0)?(t1-t0):0;
  st.work_us_last=work_us;
  if(work_us > st.work_us_max) st.work_us_max = work_us;

  static uint32_t last_print_ms=0;
  if((int32_t)(now_ms - last_print_ms) >= 200){
    last_print_ms=now_ms;
    uint64_t dirty=dirty_derived();

    logf(
      "[BPU2.9b-r4] tick=%lu ev(in/out/merge/drop)=%lu/%lu/%lu/%lu evQ=%u "
      "job(drawQ/ctrlQ)=%u/%u jobOut(draw/ctrl)=%lu/%lu jobDrop(draw/ctrl)=%lu/%lu mergeCtrl=%lu dirty=0x%016llX "
      "uart(sent/skipB/skipTX/bytes)=%lu/%lu/%lu/%lu flush(try/ok/drawOk/ctrlOk/partial/full)=%lu/%lu/%lu/%lu/%lu/%lu "
      "pick(s/h/t/d/aged)=%lu/%lu/%lu/%lu/%lu draw(ev/jobs/pts)=%lu/%lu/%lu "
      "degrade(drop/requeue/oldDrawDrop)=%lu/%lu/%lu work_us(last/max)=%lu/%lu\n",
      (unsigned long)st.tick,
      (unsigned long)st.ev_in,(unsigned long)st.ev_out,(unsigned long)st.ev_merge,(unsigned long)st.ev_drop,(unsigned)evq.count,
      (unsigned)drawq.count,(unsigned)ctrlq.count,
      (unsigned long)st.job_out_draw,(unsigned long)st.job_out_ctrl,
      (unsigned long)st.job_drop_draw,(unsigned long)st.job_drop_ctrl,
      (unsigned long)st.mergeCtrl,
      (unsigned long long)dirty,
      (unsigned long)st.uart_sent,(unsigned long)st.uart_skipB,(unsigned long)st.uart_skipTX,(unsigned long)st.uart_bytes,
      (unsigned long)st.flush_try,(unsigned long)st.flush_ok,(unsigned long)st.flush_drawOk,(unsigned long)st.flush_ctrlOk,
      (unsigned long)st.flush_partial,(unsigned long)st.flush_full,
      (unsigned long)st.pick_s,(unsigned long)st.pick_h,(unsigned long)st.pick_t,(unsigned long)st.pick_d,(unsigned long)st.pick_aged,
      (unsigned long)st.draw_ev,(unsigned long)st.draw_jobs,(unsigned long)st.draw_pts,
      (unsigned long)st.degrade_drop,(unsigned long)st.degrade_requeue,(unsigned long)st.oldDrawDrop,
      (unsigned long)st.work_us_last,(unsigned long)st.work_us_max
    );
  }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void bpu_setup(){
  LOG.begin(LOG_BAUD);
  delay(200);

  OUT.begin(OUT_BAUD, SERIAL_8N1, OUT_RX_PIN, OUT_TX_PIN);
  delay(50);

  logf("\nBPU v2.9b-r4 boot\n");
  logf("LOG: Serial @%lu\n", (unsigned long)LOG_BAUD);
  logf("OUT: Serial1 TX=GPIO%d @%lu\n", OUT_TX_PIN, (unsigned long)OUT_BAUD);
  logf("SENSOR_MS=%lu HB_MS=%lu TELEM_MS=%lu TICK_MS=%lu\n",
       (unsigned long)SENSOR_MS,(unsigned long)HB_MS,(unsigned long)TELEM_MS,(unsigned long)TICK_MS);
  logf("EVT_QN=%u DRAWQ=%u CTRLQ=%u TX_BUDGET=%u OUT_MIN_FREE=%d DRAW_STRESS=%s DRAW_PER_TICK=%u DRAW_MAXPTS/JOB=%u\n",
       (unsigned)EVT_QN,(unsigned)DRAWQ_N,(unsigned)CTRLQ_N,(unsigned)TX_BUDGET_BYTES,(int)OUT_MIN_FREE,
       ENABLE_DRAW_STRESS?"ON":"OFF",(unsigned)DRAW_PER_TICK,(unsigned)DRAW_MAXPTS_PER_JOB);

  uint32_t now=millis();
  t_next_sensor=now+10;
  t_next_hb=now+50;
  t_next_telem=now+200;
}

void bpu_loop(){
  static uint32_t last_tick_ms=0;
  uint32_t now=millis();

  if((int32_t)(now - last_tick_ms) >= (int32_t)TICK_MS){
    last_tick_ms += TICK_MS;
    bpu_tick(now);
  }
  delay(1);
}
