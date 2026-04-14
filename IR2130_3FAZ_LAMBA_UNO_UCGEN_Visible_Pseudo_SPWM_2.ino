/*  ____       __________              _ __  
   / __ \_____/_  __/ __ \____  ____  (_) /__
  / / / / ___/ / / / /_/ / __ \/ __ \/ / //_/
 / /_/ / /  _ / / / _, _/ /_/ / / / / / ,<   
/_____/_/  (_)_/ /_/ |_|\____/_/ /_/_/_/|_|
                                        &Codex

Mart/2026/İzmir/Türkiye
IR2130 ve UNO için, 3 faz delta ampul DEMO2
pseudo spwm, 2 µs @16 MHz yazılım deadtime, 12VDCBUS

UNO Pins: U_HIGH=D9(OC1A/PB1),  U_LOW=D8(PB0)
          V_HIGH=D10(OC1B/PB2), V_LOW=D7(PD7)
          W_HIGH=D11(PB3),      W_LOW=D6(PD6)
*/
const uint8_t sineTable[64] = {
  128, 140, 153, 165, 177, 188, 199, 209,
  218, 226, 234, 240, 245, 250, 253, 254,
  255, 254, 253, 250, 245, 240, 234, 226,
  218, 209, 199, 188, 177, 165, 153, 140,
  128, 115, 102, 90, 78, 67, 56, 46,
  37, 29, 21, 15, 10, 5, 2, 1,
  0, 1, 2, 5, 10, 15, 21, 29,
  37, 46, 56, 67, 78, 90, 102, 115
};

const uint8_t U_H_MASK = _BV(PB1), U_L_MASK = _BV(PB0);
const uint8_t V_H_MASK = _BV(PB2), V_L_MASK = _BV(PD7);
const uint8_t W_H_MASK = _BV(PB3), W_L_MASK = _BV(PD6);

const uint16_t PERIOD_TICKS = 400;  // 16 MHz / 400 = 40 kHz
//const uint16_t PERIOD_TICKS = 1600;  // 16 MHz / 1600 = 10 kHz
const uint16_t DUTY_MAX = 350;  // %87 duty
//const uint16_t DUTY_MAX     = 800;    // %50 duty
const uint16_t DEAD_US = 2;  // ~2 µs dead-time
const uint16_t OFF_V = PERIOD_TICKS / 3;
const uint16_t OFF_W = (PERIOD_TICKS * 2) / 3;

volatile uint16_t duty_u = DUTY_MAX, duty_v = 0, duty_w = 0;

struct PhaseState {
  bool high_on;
};
PhaseState u{ false }, v{ false }, w{ false };

static uint8_t idx = 0;
static uint8_t divider = 0;

void setup() {
  DDRB |= U_H_MASK | U_L_MASK | V_H_MASK | W_H_MASK;  // PB0,1,2,3 çıkış
  DDRD |= V_L_MASK | W_L_MASK;                        // PD7,6 çıkış

  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);  // CTC, prescaler 1
  OCR1A = PERIOD_TICKS - 1;
  TIMSK1 = _BV(OCIE1A);  // OCR1A interrupt
}

static inline void deadtime_delay() {
  // 32 nop ≈ 2 µs @16 MHz
  asm volatile(
    "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"
    "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
}

static inline void drive_phase(bool want_high, uint8_t h_mask, uint8_t l_mask,
                               volatile uint8_t &h_port, volatile uint8_t &l_port,
                               PhaseState &st) {
  if (want_high != st.high_on) {
    // her iki kolu kapat: LOW seviyeye çekerek (aktif-low giriş)
    h_port |= h_mask;  // 1 = kapalı
    l_port |= l_mask;
    deadtime_delay();
    if (want_high) {
      h_port &= ~h_mask;  // 0 = açık
    } else {
      l_port &= ~l_mask;  // 0 = açık
    }
    st.high_on = want_high;
  }
}

ISR(TIMER1_COMPA_vect) {
  static uint16_t t = 0;
  uint16_t v_pos = t + OFF_V;
  if (v_pos >= PERIOD_TICKS) v_pos -= PERIOD_TICKS;
  uint16_t w_pos = t + OFF_W;
  if (w_pos >= PERIOD_TICKS) w_pos -= PERIOD_TICKS;

  bool u_high = (t < duty_u);
  bool v_high = (v_pos < duty_v);
  bool w_high = (w_pos < duty_w);

  drive_phase(u_high, U_H_MASK, U_L_MASK, PORTB, PORTB, u);
  drive_phase(v_high, V_H_MASK, V_L_MASK, PORTB, PORTD, v);
  drive_phase(w_high, W_H_MASK, W_L_MASK, PORTB, PORTD, w);

  if (++t >= PERIOD_TICKS) t = 0;
}

void setDuty(uint16_t du, uint16_t dv, uint16_t dw) {
  noInterrupts();
  duty_u = du;
  duty_v = dv;
  duty_w = dw;
  interrupts();
}

void fadePhase(uint8_t phase) {
  const uint16_t step = DUTY_MAX / 80;  // 80 adımda yumuşak artış/azalış
  //const uint16_t step = DUTY_MAX / 20;  // 20 adımda yumuşak artış/azalış
  for (uint16_t d = 0; d <= DUTY_MAX; d += step) {
    if (phase == 0) setDuty(d, 0, 0);
    else if (phase == 1) setDuty(0, d, 0);
    else setDuty(0, 0, d);
    delay(3);
    //delay(20);
  }
  for (int d = DUTY_MAX; d >= 0; d -= step) {
    if (phase == 0) setDuty(d, 0, 0);
    else if (phase == 1) setDuty(0, d, 0);
    else setDuty(0, 0, d);
    delay(20);
  }
}

void loop() {

  divider++;

  if (divider >= 6) {  //  büyüt → daha yavaş
    divider = 0;
    idx++;
    if (idx >= 64) idx = 0;
  }

  uint8_t u = sineTable[idx];
  uint8_t v = sineTable[(idx + 21) % 64];
  uint8_t w = sineTable[(idx + 42) % 64];

  uint16_t du = (uint32_t)u * DUTY_MAX / 255;
  uint16_t dv = (uint32_t)v * DUTY_MAX / 255;
  uint16_t dw = (uint32_t)w * DUTY_MAX / 255;

  setDuty(du, dv, dw);

  delay(2);
}
/*
📌//___:
📩 bilgi@ronaer.com
🟩 https://whatsapp.com/channel/0029VaxtFPiLSmbgUryuGs0E
📷 https://www.instagram.com/dr.tronik2023/   
📺 www.youtube.com/c/DrTRonik 
👉 https://www.pcbway.com/project/member/shareproject/?bmbno=A0E12018-0BBC-4C
*/
