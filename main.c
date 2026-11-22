/*
 * EL334 Mini-Project  (v2)
 * Flappy Bird บน LCD 16x2 (I2C) + 7-Segment 4 หลัก   FRDM-K64F + Multi-Function Shield
 *
 * สิ่งที่เพิ่มจากเวอร์ชันก่อน
 *   1) เสียงแบบมีมิติ : pitch glide (สไลด์ความถี่), arpeggio จำลองคอร์ด (เมเจอร์/ไมเนอร์),
 *                       vibrato, envelope ค่อยๆ เบาลง, จังหวะตัดเสียง (articulation),
 *                       ระดับเสียงคะแนนสูงขึ้นตามคะแนน, จิงเกิลเปิดเกม/จบเกม/ทำสถิติใหม่
 *      ควบคุมความดังด้วย duty cycle ของคลื่นสี่เหลี่ยม (OFF / LOW / MID / HIGH)
 *   2) ปรับระดับความยากด้วยปุ่ม S1 (A1 = PTB3) ในเมนู (วน EASY -> MID -> HARD)
 *   3) ไฟ RGB บนบอร์ด FRDM-K64F (R=PTB22, G=PTE26, B=PTB21 Active-Low) ทำเป็นไฟเอฟเฟกต์
 *      ใช้ software PWM 16 ระดับด้วย PIT2 (8 kHz) -> มีสีรุ้งในเมนู, หายใจตามเลเวลตอนเล่น,
 *      แฟลชตอนกระพือปีก/ได้คะแนน/เลเวลอัป/ชน, สีไฟจราจรตอนนับถอยหลัง
 *   4) เมนู 3 หน้า : MAIN (ความยาก+ความดัง) / HI-SCORE (คะแนนสูงสุดแยกตามความยาก) /
 *                    SOUND (แสดงแถบความดัง)  ปรับความดังด้วยปุ่ม SW3 (+) / SW2 (-) บนบอร์ด FRDM
 *
 * ปุ่มในเมนู   S1 = เปลี่ยนความยาก   S2 = เปลี่ยนหน้าเมนู   S3 = เริ่มเล่น   (ปุ่มบน Shield)
 *              SW3 (PTA4) = เพิ่มความดัง   SW2 (PTC6) = ลดความดัง   (ปุ่มบนบอร์ด FRDM, ใช้ได้ทุกหน้าเมนู)
 * ปุ่มตอนเล่น  S1/S2/S3 ปุ่มใดก็ได้ = นกกระพือปีก
 * LED D1-D4    : ในเมนู = ระดับความยาก (1-3 ดวง) / ตอนเล่น = เลเวลความเร็ว (เพิ่มทุก 5 คะแนน)
 * 7-Segment    : คะแนน / ข้อความเมนู
 *
 * ลำดับ : splash -> เมนู -> นับถอยหลัง 3 วินาที -> เล่น -> GAME OVER -> กลับเมนู
 * หมายเหตุ : High Score เก็บใน RAM (ดับไฟแล้วรีเซ็ต)
 *
 * Timer/Counter : PIT0 (2 ms: สแกน 7-Segment + นับเวลา + เดินเสียง + เดินไฟเอฟเฟกต์)
 *                 PIT1 (สร้างคลื่นเสียงแบบปรับ duty ได้)  PIT2 (PWM ไฟ RGB)
 * Interrupt     : PORTB falling edge (S1=PTB3, S2=PTB10, S3=PTB11)
 * ADC           : ADC0 12-bit ที่ PTB2 (A0)
 * I2C (LCD)     : bit-bang ค้นหาขา/แอดเดรสของ LCD เอง (เหมือนเดิม)
 *
 * Pin map (Arduino header -> K64F)
 *   D3  = PTA1   Buzzer (ผ่านทรานซิสเตอร์ PNP Q1)
 *   D4  = PTB23  74HC595 LATCH     D7 = PTC3 CLK     D8 = PTC12 DATA
 *   D10 = PTD0   LED D4            D11 = PTC0  LED D3
 *   D12 = PTC7   LED D2            D13 = PTC5  LED D1     (LED เป็น Active-Low)
 *   A0  = PTB2   Potentiometer
 *   A1  = PTB3 (S1)   A2 = PTB10 (S2)   A3 = PTB11 (S3)
 *   RGB บนบอร์ด FRDM : R = PTB22, G = PTE26, B = PTB21  (แก้ได้ที่ define ด้านล่าง)
 */

#include <MK64F12.h>
#include <stdint.h>

#define MASK(x) (1UL << (x))

/* ------------------------------ Pins ------------------------------------- */
#define BUZZER_PIN      1    /* PTA1  */
#define SEG_LATCH_PIN   23   /* PTB23 */
#define SEG_CLK_PIN     3    /* PTC3  */
#define SEG_DATA_PIN    12   /* PTC12 */
#define LED_D1_PIN      5    /* PTC5  */
#define LED_D2_PIN      7    /* PTC7  */
#define LED_D3_PIN      0    /* PTC0  */
#define LED_D4_PIN      0    /* PTD0  */
#define SW_S1_PIN       3    /* PTB3  */
#define SW_S2_PIN       10   /* PTB10 */
#define SW_S3_PIN       11   /* PTB11 */
#define POT_ADC_CH      12   /* PTB2 = ADC0_SE12 */
#define OB_SW3_PIN      4    /* PTA4 : ปุ่ม SW3 บนบอร์ด FRDM (เพิ่มความดัง) */
#define OB_SW2_PIN      6    /* PTC6 : ปุ่ม SW2 บนบอร์ด FRDM (ลดความดัง)   */

#define RGB_R_PIN       22   /* PTB22 (Active-Low) */
#define RGB_B_PIN       21   /* PTB21 (Active-Low) */
#define RGB_G_PIN       26   /* PTE26 (Active-Low) */

#define DEBOUNCE_MS     60
#define POT_HYST        100
#define SOUND_ENABLED   1    /* ตั้งเป็น 0 เพื่อปิดเสียงทั้งหมดตอนทดสอบ */

/* ------------------------------ Game config ------------------------------ */
#define WORLD_H         16
#define BIRD_COL        3
#define BIRD_H          4
#define FIX             4
#define GRAVITY         2
#define FLAP_V          (-10)
#define MAX_FALL        12
#define MAX_PIPES       4
#define LEVEL_SCORE     5
#define MAX_LEVEL       4
#define FRAME_DEC       7
#define FRAME_MIN       45

/* ตามระดับความยาก [EASY, MID, HARD] */
static const uint16_t FRAME_MS[3]    = { 90, 75, 62 };
static const uint8_t  PIPE_EVERY[3]  = {  4,  3,  3 };
static const uint8_t  SPACING_MIN[3] = {  8,  7,  6 };
static const uint8_t  SPACING_RND[3] = {  3,  3,  2 };
static const uint8_t  MID_PCT[3]     = { 50, 30, 15 };

static const int8_t PIPE_BLOCK[3][2][2] = {
    { { 8, 16 }, { 0, 0 } },
    { { 0,  8 }, { 0, 0 } },
    { { 0,  3 }, { 13, 16 } }
};

#define G_BIRD_TOP  0
#define G_BIRD_BOT  1
#define G_BLOCK     2
#define G_PTOP      3
#define G_PBOT      4

static const uint8_t PIPE_CELL[3][2] = {
    { ' ', G_BLOCK }, { G_BLOCK, ' ' }, { G_PTOP, G_PBOT }
};
static const uint8_t GLYPH_BLOCK[8] = { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F };
static const uint8_t GLYPH_PTOP[8]  = { 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t GLYPH_PBOT[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F };
static const uint8_t BIRD_A[BIRD_H] = { 0x0E, 0x1F, 0x1E, 0x08 };
static const uint8_t BIRD_B[BIRD_H] = { 0x13, 0x1F, 0x0E, 0x04 };

/* ---------------------- 7-Segment (Active-Low) --------------------------- */
static const uint8_t DIGIT_SELECT[4] = { 0xF1, 0xF2, 0xF4, 0xF8 };
static const uint8_t DIGIT_SEG[10]   = { 0xC0, 0xF9, 0xA4, 0xB0, 0x99,
                                         0x92, 0x82, 0xF8, 0x80, 0x90 };

/* ============================ ระบบเสียงแบบมีมิติ ============================ */
/*
 * 1 โน้ต = { f0 (Hz เริ่มต้น), f1 (Hz สิ้นสุด -> สไลด์ความถี่), ms (ระยะเวลา), fx (เอฟเฟกต์) }
 *   FXN_DECAY : ความดังค่อยๆ ลดลงตลอดโน้ต (เสียงนุ่ม/ก้อง)
 *   FXN_VIB   : vibrato ~ +-2.5 %
 *   FXN_MAJ   : arpeggio สลับ 1 : 5/4 : 3/2 เร็วๆ -> ฟังเหมือนคอร์ดเมเจอร์
 *   FXN_MIN   : arpeggio สลับ 1 : 6/5 : 3/2 -> คอร์ดไมเนอร์ (เศร้า/หนัก)
 *   FXN_LEG   : เชื่อมโน้ต (ไม่ตัดช่วงท้ายโน้ต)
 */
typedef struct {
    uint16_t f0;
    uint16_t f1;
    uint16_t ms;
    uint8_t  fx;
} Note_t;

#define FXN_DECAY 0x01
#define FXN_VIB   0x02
#define FXN_MAJ   0x04
#define FXN_MIN   0x08
#define FXN_LEG   0x10

/* ความดัง 0..3 -> duty cycle (%) ของคลื่นสี่เหลี่ยม : เล็ก = เบา */
static const uint8_t VOL_DUTY[4] = { 0, 7, 18, 50 };
static const uint8_t MAJ_NUM[3]  = { 4, 5, 6 };      /* /4  */
static const uint8_t MIN_NUM[3]  = { 10, 12, 15 };   /* /10 */

static const Note_t SFX_FLAP[]  = { { 520, 1500, 45, FXN_DECAY | FXN_LEG },      /* วู้บ ขึ้น */
                                    { 1500, 1900, 30, FXN_DECAY } };
static const Note_t SFX_LEVEL[] = { { 523, 523, 90, FXN_MAJ | FXN_DECAY },
                                    { 659, 659, 90, FXN_MAJ | FXN_DECAY },
                                    { 784, 784, 90, FXN_MAJ | FXN_DECAY },
                                    { 1047, 1047, 280, FXN_MAJ | FXN_VIB | FXN_DECAY } };
static const Note_t SFX_HIT[]   = { { 900, 220, 160, FXN_DECAY | FXN_LEG },      /* ปึ้ก + ตุบ */
                                    { 220, 90, 320, FXN_DECAY | FXN_MIN | FXN_VIB } };
static const Note_t SFX_BEEP3[] = { { 660, 660, 140, FXN_DECAY } };
static const Note_t SFX_BEEP2[] = { { 784, 784, 140, FXN_DECAY } };
static const Note_t SFX_BEEP1[] = { { 988, 988, 140, FXN_DECAY } };
static const Note_t SFX_GO[]    = { { 1047, 1047, 70, FXN_MAJ },
                                    { 1568, 1568, 260, FXN_MAJ | FXN_VIB | FXN_DECAY } };
static const Note_t SFX_START[] = { { 523, 784, 90, FXN_MAJ | FXN_LEG },
                                    { 784, 1047, 90, FXN_MAJ | FXN_LEG },
                                    { 1047, 1047, 220, FXN_MAJ | FXN_VIB | FXN_DECAY } };
static const Note_t SFX_LV_E[]  = { { 392, 523, 70, FXN_DECAY } };
static const Note_t SFX_LV_M[]  = { { 523, 659, 40, FXN_LEG }, { 659, 784, 70, FXN_DECAY | FXN_MAJ } };
static const Note_t SFX_LV_H[]  = { { 659, 880, 40, FXN_LEG }, { 880, 1175, 40, FXN_LEG },
                                    { 1175, 1175, 90, FXN_DECAY | FXN_MIN | FXN_VIB } };
static const Note_t SFX_PAGE[]  = { { 1000, 1500, 35, FXN_DECAY } };
static const Note_t SFX_VOLP[]  = { { 880, 880, 60, FXN_DECAY },
                                    { 1319, 1319, 130, FXN_DECAY | FXN_MAJ } };
static const Note_t SFX_OVER[]  = { { 740, 740, 160, FXN_MIN | FXN_DECAY },          /* โน้ตไล่ลงทีละขั้น */
                                    { 622, 622, 160, FXN_MIN | FXN_DECAY },
                                    { 554, 554, 160, FXN_MIN | FXN_DECAY },
                                    { 466, 466, 260, FXN_MIN | FXN_DECAY | FXN_LEG },
                                    { 466, 233, 520, FXN_MIN | FXN_VIB | FXN_DECAY },   /* ไหลลงทั้งอ็อกเทฟ */
                                    { 110, 65, 700, FXN_MIN | FXN_VIB | FXN_DECAY } };  /* ทุ้มต่ำสั่นๆ จางหาย */
static const Note_t SFX_BEST[]  = { { 523, 523, 100, FXN_MAJ },
                                    { 659, 659, 100, FXN_MAJ },
                                    { 784, 784, 100, FXN_MAJ },
                                    { 1047, 1047, 100, FXN_MAJ },
                                    { 1319, 1319, 450, FXN_MAJ | FXN_VIB | FXN_DECAY } };

static Note_t dynNote[2];                               /* เสียงได้คะแนน (สร้างตอนรัน) */

typedef struct {
    const Note_t *data;
    uint8_t  len;
    uint8_t  pos;
    uint8_t  active;
    Note_t   cur;
    int32_t  elapsed;
    int32_t  dur;
} Seq_t;

static Seq_t sfx;
static uint8_t sfxPrio = 0;
static volatile uint8_t volumeLevel = 2;                /* 0=OFF 1=LOW 2=MID 3=HIGH */

#define SFX_PLAY(arr, prio) Sfx_Start((arr), (uint8_t)(sizeof(arr) / sizeof((arr)[0])), (prio))

/* ------------------------------- ตัวแปรส่วนกลาง -------------------------- */
static uint32_t busClkHz = 20971520u;
static uint32_t i2cLoops = 20;

static volatile uint32_t msTicks = 0;
static volatile uint8_t  dispBuf[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
static volatile uint8_t  scanIndex = 0;
static volatile uint8_t  evS1 = 0, evS2 = 0, evS3 = 0;
static volatile uint32_t lastPress[3] = { 0, 0, 0 };

typedef struct {
    int8_t  x;
    uint8_t type;
    uint8_t active;
    uint8_t scored;
} Pipe_t;

static Pipe_t   pipes[MAX_PIPES];
static int16_t  birdY = 0, birdV = 0;
static uint16_t score = 0;
static uint16_t best[3] = { 0, 0, 0 };                  /* High Score แยกตามความยาก */
static uint8_t  level = 1, difficulty = 0;
static uint32_t frameMs = 90;
static uint8_t  sinceSpawn = 0, nextSpacing = 5;
static uint32_t rngState = 12345u;

/* ============================== Buzzer / Tone ============================== */

/* ตัด Buzzer : ปิดขา PTA1 ทั้งหมด (High-Z) เพราะ Q1 เป็น PNP ขับด้วย 3.3 V ไม่ดับสนิท */
void Mute_Buzzer_HighZ(void)
{
    PTA->PCOR  = MASK(BUZZER_PIN);
    PTA->PDDR &= ~MASK(BUZZER_PIN);
    PORTA->PCR[BUZZER_PIN] = PORT_PCR_MUX(0);
}

void Init_Buzzer(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    Mute_Buzzer_HighZ();
}

/*
 * คลื่นเสียงจาก PIT1 : สลับขา PTA1 ระหว่าง "ON" (output ต่ำ -> Q1 นำกระแส)
 * กับ "OFF" (input -> Q1 ตัด) โดยเวลา ON/OFF ไม่เท่ากันได้ => ปรับความดังด้วย duty
 */
static volatile uint32_t toneOnLd = 100, toneOffLd = 100;
static volatile uint8_t  toneRun = 0, toneOn = 0;

static void Tone_Stop(void)
{
    PIT->CHANNEL[1].TCTRL = 0;
    PIT->CHANNEL[1].TFLG  = PIT_TFLG_TIF_MASK;
    NVIC_ClearPendingIRQ(PIT1_IRQn);
    Mute_Buzzer_HighZ();
    toneRun = 0;
}

/* freq (Hz), duty (% ที่ Q1 นำกระแส) เรียกจาก PIT0 ISR */
static void Tone_Update(uint32_t freq, uint32_t duty)
{
    uint32_t period, on, off;

    if (!SOUND_ENABLED || freq < 40u || duty == 0u) {
        if (toneRun) Tone_Stop();
        return;
    }
    period = busClkHz / freq;
    on  = (period * duty) / 100u;
    if (on < 2u) on = 2u;
    off = (period > on) ? (period - on) : 2u;
    if (off < 2u) off = 2u;
    toneOnLd  = on - 1u;
    toneOffLd = off - 1u;

    if (!toneRun) {
        PORTA->PCR[BUZZER_PIN] = PORT_PCR_MUX(1);
        PTA->PCOR = MASK(BUZZER_PIN);
        PTA->PDDR |= MASK(BUZZER_PIN);                  /* เริ่มที่ช่วง ON */
        toneOn = 1;
        PIT->CHANNEL[1].TFLG  = PIT_TFLG_TIF_MASK;
        PIT->CHANNEL[1].LDVAL = toneOnLd;
        PIT->CHANNEL[1].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
        PIT->CHANNEL[1].LDVAL = toneOffLd;              /* มีผลตอนรีโหลดรอบถัดไป (ช่วง OFF) */
        toneRun = 1;
    }
}

void PIT1_IRQHandler(void)
{
    PIT->CHANNEL[1].TFLG = PIT_TFLG_TIF_MASK;
    if (toneOn) {                                       /* เข้าช่วง OFF */
        PTA->PDDR &= ~MASK(BUZZER_PIN);
        toneOn = 0;
        PIT->CHANNEL[1].LDVAL = toneOnLd;               /* ใช้ตอนรีโหลดเข้าช่วง ON */
    } else {                                            /* เข้าช่วง ON */
        PTA->PDDR |= MASK(BUZZER_PIN);
        toneOn = 1;
        PIT->CHANNEL[1].LDVAL = toneOffLd;
    }
}

/* เริ่มเล่นเสียง : เสียง priority ต่ำกว่าไม่ตัดเสียงที่สำคัญกว่าที่กำลังเล่น */
static void Sfx_Start(const Note_t *data, uint8_t len, uint8_t prio)
{
    __disable_irq();
    if (!(sfx.active && prio < sfxPrio)) {
        sfxPrio     = prio;
        sfx.data    = data;
        sfx.len     = len;
        sfx.pos     = 0;
        sfx.elapsed = 0;
        sfx.dur     = 0;
        sfx.active  = 1;
    }
    __enable_irq();
}

static uint8_t Sfx_IsActive(void)
{
    return *(volatile uint8_t *)&sfx.active;
}

/* เสียงได้คะแนน : ยิ่งคะแนนสูง ระดับเสียงยิ่งสูง + ท้ายเสียงเป็นคอร์ดแวววาว */
static void Sfx_ScorePing(uint16_t sc)
{
    uint16_t base = (uint16_t)(988u + (uint32_t)((sc > 12u) ? 12u : sc) * 30u);
    __disable_irq();
    dynNote[0].f0 = base;               dynNote[0].f1 = base;
    dynNote[0].ms = 45;                 dynNote[0].fx = FXN_LEG;
    dynNote[1].f0 = (uint16_t)(base * 3u / 2u);
    dynNote[1].f1 = dynNote[1].f0;
    dynNote[1].ms = 230;                dynNote[1].fx = FXN_DECAY | FXN_MAJ | FXN_VIB;
    __enable_irq();
    Sfx_Start(dynNote, 2, 2);
}

/* เดินเสียง ทุก 2 ms (เรียกจาก PIT0 ISR) */
static void Audio_Tick(void)
{
    const Note_t *n;
    int32_t f, t, duty, env, tri;
    uint8_t idx;

    if (!sfx.active) return;

    if (sfx.elapsed >= sfx.dur) {
        if (sfx.pos >= sfx.len) {
            sfx.active = 0;
            Tone_Stop();
            return;
        }
        sfx.cur     = sfx.data[sfx.pos++];
        sfx.elapsed = 0;
        sfx.dur     = (sfx.cur.ms < 4) ? 4 : sfx.cur.ms;
    }

    n = &sfx.cur;
    t = (sfx.elapsed * 256) / sfx.dur;                              /* 0..255 */
    f = (int32_t)n->f0 + (((int32_t)n->f1 - (int32_t)n->f0) * t) / 256;   /* glide */

    if (n->fx & FXN_VIB) {                                          /* vibrato ~6 Hz */
        idx = (uint8_t)((sfx.elapsed / 20) & 15);
        tri = (idx < 8) ? ((int32_t)idx - 4) : (12 - (int32_t)idx);
        f += (f * tri) / 150;
    }
    if (n->fx & FXN_MAJ) {                                          /* arpeggio = คอร์ด */
        idx = (uint8_t)((sfx.elapsed / 10) % 3);
        f = (f * MAJ_NUM[idx]) / 4;
    } else if (n->fx & FXN_MIN) {
        idx = (uint8_t)((sfx.elapsed / 10) % 3);
        f = (f * MIN_NUM[idx]) / 10;
    }

    duty = VOL_DUTY[volumeLevel & 3];
    if (n->fx & FXN_DECAY) {                                        /* envelope */
        env  = 256 - (t * 200) / 256;                               /* 256 -> 56 */
        duty = (duty * env) / 256;
        if (duty < 1 && VOL_DUTY[volumeLevel & 3] > 0) duty = 1;
    }
    if (!(n->fx & FXN_LEG) && (sfx.dur - sfx.elapsed) <= (sfx.dur / 8))
        duty = 0;                                                   /* ตัดท้ายโน้ตให้เป็นจังหวะ */

    Tone_Update((uint32_t)f, (uint32_t)duty);
    sfx.elapsed += 2;
}

/* ============================ ไฟ RGB เอฟเฟกต์ ============================== */

#define FXM_OFF      0
#define FXM_RAINBOW  1
#define FXM_SOLID    2
#define FXM_BREATH   3

static volatile uint8_t  fxMode = FXM_OFF;
static volatile uint8_t  fxBase[3] = { 0, 0, 0 };         /* 0..15 */
static volatile uint16_t fxPeriod = 1000;
static volatile uint8_t  fxHueSpeed = 3;
static volatile uint8_t  fxFlashRGB[3] = { 0, 0, 0 };
static volatile uint16_t fxFlashAmt = 0;                  /* 0..256 */
static volatile uint8_t  fxFlashDecay = 4;
static uint32_t fxHueAcc = 0, fxT = 0;
static volatile uint8_t rgbLvl[3] = { 0, 0, 0 };          /* ระดับสุดท้ายที่ PIT2 ใช้ทำ PWM */
static uint8_t pwmCnt = 0;

static void Fx_Off(void)                       { fxMode = FXM_OFF; }
static void Fx_Rainbow(uint8_t speed)          { fxHueSpeed = speed; fxMode = FXM_RAINBOW; }
static void Fx_Solid(uint8_t r, uint8_t g, uint8_t b)
{
    fxBase[0] = r; fxBase[1] = g; fxBase[2] = b;
    fxMode = FXM_SOLID;
}
static void Fx_Breath(uint8_t r, uint8_t g, uint8_t b, uint16_t periodMs)
{
    fxBase[0] = r; fxBase[1] = g; fxBase[2] = b;
    fxPeriod = (periodMs < 100) ? 100 : periodMs;
    fxMode = FXM_BREATH;
}
static void Fx_Flash(uint8_t r, uint8_t g, uint8_t b, uint16_t amt, uint8_t decay)
{
    fxFlashRGB[0] = r; fxFlashRGB[1] = g; fxFlashRGB[2] = b;
    fxFlashDecay = (decay == 0) ? 1 : decay;
    fxFlashAmt = (amt > 256) ? 256 : amt;
}

static void Hue_To_Rgb(uint16_t h, uint8_t v, uint8_t *c)         /* h = 0..1535 */
{
    uint8_t seg = (uint8_t)(h >> 8), f = (uint8_t)(h & 0xFF), r, g, b;
    switch (seg) {
    case 0:  r = 255;     g = f;       b = 0;       break;
    case 1:  r = 255 - f; g = 255;     b = 0;       break;
    case 2:  r = 0;       g = 255;     b = f;       break;
    case 3:  r = 0;       g = 255 - f; b = 255;     break;
    case 4:  r = f;       g = 0;       b = 255;     break;
    default: r = 255;     g = 0;       b = 255 - f; break;
    }
    c[0] = (uint8_t)((r * v) / 255);
    c[1] = (uint8_t)((g * v) / 255);
    c[2] = (uint8_t)((b * v) / 255);
}

/* เดินเอฟเฟกต์ ทุก 2 ms (เรียกจาก PIT0 ISR) */
static void Fx_Tick(void)
{
    uint8_t c[3] = { 0, 0, 0 };
    uint8_t i;
    uint32_t half, p, tri, scale;
    int32_t v;

    fxT += 2;
    switch (fxMode) {
    case FXM_RAINBOW:
        fxHueAcc += fxHueSpeed;
        if (fxHueAcc >= 1536u * 8u) fxHueAcc -= 1536u * 8u;
        Hue_To_Rgb((uint16_t)(fxHueAcc >> 3), 12, c);
        break;
    case FXM_SOLID:
        for (i = 0; i < 3; i++) c[i] = fxBase[i];
        break;
    case FXM_BREATH:
        half  = fxPeriod / 2u;
        p     = fxT % fxPeriod;
        tri   = (p < half) ? p : (fxPeriod - p);
        scale = 3u + (tri * 12u) / half;                    /* 3..15 */
        for (i = 0; i < 3; i++) c[i] = (uint8_t)(((uint32_t)fxBase[i] * scale) / 15u);
        break;
    default:
        break;
    }

    if (fxFlashAmt) {                                       /* แฟลชทับ (บวกแล้วจำกัดที่ 15) */
        for (i = 0; i < 3; i++) {
            v = (int32_t)c[i] + ((int32_t)fxFlashRGB[i] * (int32_t)fxFlashAmt) / 256;
            c[i] = (uint8_t)((v > 15) ? 15 : v);
        }
        fxFlashAmt = (fxFlashAmt > fxFlashDecay) ? (uint16_t)(fxFlashAmt - fxFlashDecay) : 0;
    }

    for (i = 0; i < 3; i++)                                 /* gamma ง่ายๆ ให้การไล่แสงนุ่มตา */
        rgbLvl[i] = (uint8_t)(((uint32_t)c[i] * c[i] + 14u) / 15u);
}

/* PIT2 : PWM ซอฟต์แวร์ 16 ระดับ ที่ 8 kHz (-> ~500 Hz ต่อรอบ) */
void PIT2_IRQHandler(void)
{
    uint32_t setB = 0, clrB = 0;
    PIT->CHANNEL[2].TFLG = PIT_TFLG_TIF_MASK;
    pwmCnt = (uint8_t)((pwmCnt + 1u) & 15u);

    if (pwmCnt < rgbLvl[0]) clrB |= MASK(RGB_R_PIN); else setB |= MASK(RGB_R_PIN);
    if (pwmCnt < rgbLvl[2]) clrB |= MASK(RGB_B_PIN); else setB |= MASK(RGB_B_PIN);
    PTB->PSOR = setB;
    PTB->PCOR = clrB;

    if (pwmCnt < rgbLvl[1]) PTE->PCOR = MASK(RGB_G_PIN);
    else                    PTE->PSOR = MASK(RGB_G_PIN);
}

/* ================================ 7-Segment ================================ */

static void Shift_Byte(uint8_t value)
{
    int8_t i;
    for (i = 7; i >= 0; i--) {
        if (value & MASK(i)) PTC->PSOR = MASK(SEG_DATA_PIN);
        else                 PTC->PCOR = MASK(SEG_DATA_PIN);
        PTC->PSOR = MASK(SEG_CLK_PIN);
        PTC->PCOR = MASK(SEG_CLK_PIN);
    }
}

void Show_7Segment_Raw(uint8_t pos, uint8_t segByte)
{
    PTB->PCOR = MASK(SEG_LATCH_PIN);
    Shift_Byte(segByte);
    Shift_Byte(DIGIT_SELECT[pos]);
    PTB->PSOR = MASK(SEG_LATCH_PIN);
}

void Delay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) { }
}

uint8_t Char_To_Seg(char c)
{
    if (c >= '0' && c <= '9') return DIGIT_SEG[c - '0'];
    switch (c) {
    case ' ': return 0xFF;
    case 'A': return 0x88;
    case 'E': return 0x86;
    case 'S': return 0x92;
    case 'd': return 0xA1;
    case 'y': return 0x91;
    case 'r': return 0xAF;
    case 'H': return 0x89;
    case 'M': return 0xC8;
    case 'i': return 0xFB;
    case 'n': return 0xAB;
    default:  return 0xBF;     /* '-' */
    }
}

void Show_Text4(const char *t)
{
    uint8_t i;
    for (i = 0; i < 4; i++) dispBuf[i] = Char_To_Seg(t[i]);
}

void Show_Number(uint16_t n)
{
    if (n > 9999) n = 9999;
    dispBuf[3] = DIGIT_SEG[n % 10];
    dispBuf[2] = (n >= 10)   ? DIGIT_SEG[(n / 10) % 10]   : 0xFF;
    dispBuf[1] = (n >= 100)  ? DIGIT_SEG[(n / 100) % 10]  : 0xFF;
    dispBuf[0] = (n >= 1000) ? DIGIT_SEG[(n / 1000) % 10] : 0xFF;
}

/* =============================== Interrupts ================================ */

void PIT0_IRQHandler(void)
{
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;

    Show_7Segment_Raw(scanIndex, dispBuf[scanIndex]);
    scanIndex = (scanIndex + 1) & 3;

    msTicks += 2;

    Audio_Tick();                                       /* เดินเสียง */
    Fx_Tick();                                          /* เดินไฟ RGB */
}

void PORTB_IRQHandler(void)
{
    uint32_t flags = PORTB->ISFR;
    PORTB->ISFR = flags;

    if ((flags & MASK(SW_S1_PIN)) && (msTicks - lastPress[0] > DEBOUNCE_MS)) {
        lastPress[0] = msTicks;  evS1 = 1;
    }
    if ((flags & MASK(SW_S2_PIN)) && (msTicks - lastPress[1] > DEBOUNCE_MS)) {
        lastPress[1] = msTicks;  evS2 = 1;
    }
    if ((flags & MASK(SW_S3_PIN)) && (msTicks - lastPress[2] > DEBOUNCE_MS)) {
        lastPress[2] = msTicks;  evS3 = 1;
    }
}

/* ================================== LEDs =================================== */

void Led_Set(uint8_t idx, uint8_t on)
{
    switch (idx) {
    case 0:  if (on) PTC->PCOR = MASK(LED_D1_PIN); else PTC->PSOR = MASK(LED_D1_PIN); break;
    case 1:  if (on) PTC->PCOR = MASK(LED_D2_PIN); else PTC->PSOR = MASK(LED_D2_PIN); break;
    case 2:  if (on) PTC->PCOR = MASK(LED_D3_PIN); else PTC->PSOR = MASK(LED_D3_PIN); break;
    default: if (on) PTD->PCOR = MASK(LED_D4_PIN); else PTD->PSOR = MASK(LED_D4_PIN); break;
    }
}

void Leds_All(uint8_t on)
{
    uint8_t i;
    for (i = 0; i < 4; i++) Led_Set(i, on);
}

void Leds_Level(uint8_t n)
{
    uint8_t i;
    for (i = 0; i < 4; i++) Led_Set(i, (uint8_t)(i < n));
}

/* ================================== Init =================================== */

void Init_GPIO(void)
{
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
                  SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    PORTB->PCR[SEG_LATCH_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[SEG_CLK_PIN]   = PORT_PCR_MUX(1);
    PORTC->PCR[SEG_DATA_PIN]  = PORT_PCR_MUX(1);
    PTB->PDDR |= MASK(SEG_LATCH_PIN);
    PTC->PDDR |= MASK(SEG_CLK_PIN) | MASK(SEG_DATA_PIN);
    PTB->PCOR = MASK(SEG_LATCH_PIN);
    PTC->PCOR = MASK(SEG_CLK_PIN);

    PORTC->PCR[LED_D1_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[LED_D2_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[LED_D3_PIN] = PORT_PCR_MUX(1);
    PORTD->PCR[LED_D4_PIN] = PORT_PCR_MUX(1);
    PTC->PDDR |= MASK(LED_D1_PIN) | MASK(LED_D2_PIN) | MASK(LED_D3_PIN);
    PTD->PDDR |= MASK(LED_D4_PIN);
    Leds_All(0);

    PORTB->PCR[SW_S1_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[SW_S2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB->PCR[SW_S3_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTB->PDDR &= ~(MASK(SW_S1_PIN) | MASK(SW_S2_PIN) | MASK(SW_S3_PIN));

    /* ปุ่ม SW3 / SW2 บนบอร์ด FRDM-K64F (Active-Low, อ่านแบบ polling ในเมนู) */
    PORTA->PCR[OB_SW3_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTC->PCR[OB_SW2_PIN] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTA->PDDR &= ~MASK(OB_SW3_PIN);
    PTC->PDDR &= ~MASK(OB_SW2_PIN);
}

void Init_RGB(void)                                     /* ไฟ RGB บนบอร์ด (Active-Low) ปิดไว้ก่อน */
{
    PORTB->PCR[RGB_R_PIN] = PORT_PCR_MUX(1);
    PORTB->PCR[RGB_B_PIN] = PORT_PCR_MUX(1);
    PORTE->PCR[RGB_G_PIN] = PORT_PCR_MUX(1);
    PTB->PSOR = MASK(RGB_R_PIN) | MASK(RGB_B_PIN);
    PTE->PSOR = MASK(RGB_G_PIN);
    PTB->PDDR |= MASK(RGB_R_PIN) | MASK(RGB_B_PIN);
    PTE->PDDR |= MASK(RGB_G_PIN);
}

void Init_ADC0(void)
{
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    PORTB->PCR[2] = PORT_PCR_MUX(0);
    ADC0->CFG1 = ADC_CFG1_ADIV(2) | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0) |
                 ADC_CFG1_ADLSMP_MASK;
    ADC0->SC2  = 0;
}

uint16_t Read_ADC0(void)
{
    uint32_t timeout = 200000u;
    ADC0->SC1[0] = ADC_SC1_ADCH(POT_ADC_CH);
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK) && --timeout) { }
    return (uint16_t)ADC0->R[0];
}

void Init_PIT_Timer(void)
{
    SystemCoreClockUpdate();
    busClkHz = SystemCoreClock /
               (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK) >> SIM_CLKDIV1_OUTDIV2_SHIFT) + 1);
    i2cLoops = SystemCoreClock / 1200000u;
    if (i2cLoops < 2) i2cLoops = 2;

    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
    PIT->MCR = 0;

    PIT->CHANNEL[0].LDVAL = (busClkHz / 500u) - 1u;     /* 2 ms */
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    PIT->CHANNEL[1].TCTRL = 0;                          /* PIT1 เปิดเมื่อมีเสียงเท่านั้น */

    PIT->CHANNEL[2].LDVAL = (busClkHz / 8000u) - 1u;    /* 125 us : PWM ไฟ RGB */
    PIT->CHANNEL[2].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

    NVIC_ClearPendingIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT0_IRQn);
    NVIC_ClearPendingIRQ(PIT1_IRQn);
    NVIC_EnableIRQ(PIT1_IRQn);
    NVIC_ClearPendingIRQ(PIT2_IRQn);
    NVIC_EnableIRQ(PIT2_IRQn);
}

void Init_Switch_Interrupt(void)
{
    PORTB->PCR[SW_S1_PIN] |= PORT_PCR_IRQC(0xA);
    PORTB->PCR[SW_S2_PIN] |= PORT_PCR_IRQC(0xA);
    PORTB->PCR[SW_S3_PIN] |= PORT_PCR_IRQC(0xA);
    PORTB->ISFR = 0xFFFFFFFFu;
    NVIC_ClearPendingIRQ(PORTB_IRQn);
    NVIC_EnableIRQ(PORTB_IRQn);
}

/* ============================ I2C (bit-bang) =============================== */

static const uint8_t I2C_SDA[4] = { 25, 24, 16, 17 };
static const uint8_t I2C_SCL[4] = { 24, 25, 17, 16 };

static GPIO_Type *i2cGpio = PTE;
static uint8_t sdaPin = 25, sclPin = 24;

#define SDA_LOW()   (i2cGpio->PDDR |=  MASK(sdaPin))
#define SDA_REL()   (i2cGpio->PDDR &= ~MASK(sdaPin))
#define SCL_LOW()   (i2cGpio->PDDR |=  MASK(sclPin))
#define SCL_REL()   (i2cGpio->PDDR &= ~MASK(sclPin))
#define SDA_READ()  ((i2cGpio->PDIR >> sdaPin) & 1u)

static void I2c_Delay(void)
{
    volatile uint32_t i = i2cLoops;
    while (i--) { }
}

static void I2c_Select(uint8_t idx)
{
    PORT_Type *port = (idx < 2) ? PORTE : PORTC;
    i2cGpio = (idx < 2) ? PTE : PTC;
    sdaPin = I2C_SDA[idx];
    sclPin = I2C_SCL[idx];

    port->PCR[sdaPin] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    port->PCR[sclPin] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    i2cGpio->PCOR = MASK(sdaPin) | MASK(sclPin);
    i2cGpio->PDDR &= ~(MASK(sdaPin) | MASK(sclPin));
}

static void I2c_Start(void)
{
    SDA_REL(); SCL_REL(); I2c_Delay();
    SDA_LOW(); I2c_Delay();
    SCL_LOW(); I2c_Delay();
}

static void I2c_Stop(void)
{
    SDA_LOW(); I2c_Delay();
    SCL_REL(); I2c_Delay();
    SDA_REL(); I2c_Delay();
}

static uint8_t I2c_WriteByte(uint8_t b)
{
    uint8_t i, ack;
    for (i = 0; i < 8; i++) {
        if (b & 0x80) SDA_REL(); else SDA_LOW();
        I2c_Delay();
        SCL_REL(); I2c_Delay();
        SCL_LOW();
        b = (uint8_t)(b << 1);
    }
    SDA_REL(); I2c_Delay();
    SCL_REL(); I2c_Delay();
    ack = (uint8_t)(SDA_READ() == 0);
    SCL_LOW();
    return ack;
}

static void I2c_Recover(void)
{
    uint8_t i;
    SDA_REL(); SCL_REL(); I2c_Delay();
    for (i = 0; i < 9 && !SDA_READ(); i++) {
        SCL_LOW(); I2c_Delay();
        SCL_REL(); I2c_Delay();
    }
    SDA_LOW(); I2c_Delay();
    SCL_REL(); I2c_Delay();
    SDA_REL(); I2c_Delay();
}

static uint8_t I2c_Probe(uint8_t addr7)
{
    uint8_t ack;
    I2c_Start();
    ack = I2c_WriteByte((uint8_t)(addr7 << 1));
    I2c_Stop();
    return ack;
}

/* ============================ LCD 16x2 (PCF8574) =========================== */

#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08

static uint8_t lcdAddr = 0x27, lcdOk = 0;
static uint8_t lcdBuf[2][16];
static uint8_t lcdShown[2][16];
static uint8_t glyphCache[8][8];
static uint8_t glyphValid[8];

static uint8_t Lcd_Xfer(const uint8_t *data, uint8_t n)
{
    uint8_t i, ok;
    if (!lcdOk) return 0;
    I2c_Start();
    ok = I2c_WriteByte((uint8_t)(lcdAddr << 1));
    for (i = 0; ok && i < n; i++) ok = I2c_WriteByte(data[i]);
    I2c_Stop();
    return ok;
}

static void Lcd_Send(uint8_t value, uint8_t rs)
{
    uint8_t hi = (uint8_t)((value & 0xF0) | LCD_BL | rs);
    uint8_t lo = (uint8_t)(((value << 4) & 0xF0) | LCD_BL | rs);
    uint8_t b[4];
    b[0] = (uint8_t)(hi | LCD_EN);
    b[1] = hi;
    b[2] = (uint8_t)(lo | LCD_EN);
    b[3] = lo;
    Lcd_Xfer(b, 4);
}

static void Lcd_Cmd(uint8_t c)
{
    Lcd_Send(c, 0);
    if (c <= 0x03) Delay_ms(3);
}

static void Lcd_Data(uint8_t d)
{
    Lcd_Send(d, LCD_RS);
}

static void Lcd_Nibble(uint8_t nib)
{
    uint8_t b[2];
    b[0] = (uint8_t)((nib & 0xF0) | LCD_BL | LCD_EN);
    b[1] = (uint8_t)((nib & 0xF0) | LCD_BL);
    Lcd_Xfer(b, 2);
}

static void Lcd_BufClear(void)
{
    uint8_t r, c;
    for (r = 0; r < 2; r++)
        for (c = 0; c < 16; c++) lcdBuf[r][c] = ' ';
}

static void Lcd_Clear(void)
{
    uint8_t r, c;
    Lcd_Cmd(0x01);
    for (r = 0; r < 2; r++)
        for (c = 0; c < 16; c++) { lcdBuf[r][c] = ' '; lcdShown[r][c] = ' '; }
}

static void Lcd_Init(void)
{
    Delay_ms(50);
    Lcd_Nibble(0x30); Delay_ms(5);
    Lcd_Nibble(0x30); Delay_ms(2);
    Lcd_Nibble(0x30); Delay_ms(2);
    Lcd_Nibble(0x20); Delay_ms(2);
    Lcd_Cmd(0x28);
    Lcd_Cmd(0x0C);
    Lcd_Cmd(0x06);
    Lcd_Clear();
}

static void Lcd_Puts(uint8_t r, uint8_t c, const char *s)
{
    while (*s && c < 16) lcdBuf[r][c++] = (uint8_t)*s++;
}

static void Lcd_PutNum(uint8_t r, uint8_t c, uint16_t n, uint8_t width)
{
    uint8_t i;
    for (i = width; i > 0; i--) {
        if (c + i - 1 < 16) {
            if (n > 0 || i == width) lcdBuf[r][c + i - 1] = (uint8_t)('0' + (n % 10));
            else                     lcdBuf[r][c + i - 1] = ' ';
        }
        n /= 10;
    }
}

static void Lcd_Flush(void)
{
    uint8_t r, c, start;
    if (!lcdOk) return;
    for (r = 0; r < 2; r++) {
        c = 0;
        while (c < 16) {
            if (lcdBuf[r][c] == lcdShown[r][c]) { c++; continue; }
            start = c;
            Lcd_Cmd((uint8_t)(0x80 | (r ? 0x40 : 0x00) | start));
            while (c < 16 && lcdBuf[r][c] != lcdShown[r][c]) {
                Lcd_Data(lcdBuf[r][c]);
                lcdShown[r][c] = lcdBuf[r][c];
                c++;
            }
        }
    }
}

static void Lcd_Text(const char *l0, const char *l1)
{
    Lcd_BufClear();
    Lcd_Puts(0, 0, l0);
    Lcd_Puts(1, 0, l1);
    Lcd_Flush();
}

static void Lcd_SetGlyph(uint8_t idx, const uint8_t *g)
{
    uint8_t i, changed = !glyphValid[idx];
    for (i = 0; i < 8; i++) if (glyphCache[idx][i] != g[i]) changed = 1;
    if (!changed) return;
    Lcd_Cmd((uint8_t)(0x40 | (idx << 3)));
    for (i = 0; i < 8; i++) {
        Lcd_Data(g[i]);
        glyphCache[idx][i] = g[i];
    }
    glyphValid[idx] = 1;
}

static uint8_t Lcd_Detect(void)
{
    uint8_t opt, a;
    for (opt = 0; opt < 4; opt++) {
        I2c_Select(opt);
        I2c_Recover();
        for (a = 0x20; a <= 0x3F; a++) {
            if (I2c_Probe(a)) {
                lcdAddr = a;
                lcdOk = 1;
                return 1;
            }
        }
    }
    return 0;
}

static void Lcd_Setup(void)
{
    while (!Lcd_Detect()) {
        Show_Text4("Err ");
        Fx_Solid(15, 0, 0);                             /* หา LCD ไม่เจอ -> ไฟแดง */
        Delay_ms(500);
    }
    Lcd_Init();
    Lcd_SetGlyph(G_BLOCK, GLYPH_BLOCK);
    Lcd_SetGlyph(G_PTOP,  GLYPH_PTOP);
    Lcd_SetGlyph(G_PBOT,  GLYPH_PBOT);
}

/* ================================ Game logic =============================== */

/* สี RGB ตามเลเวลความเร็ว (ฟ้า -> เขียวน้ำทะเล -> ม่วง -> ส้มแดง) */
static const uint8_t LVL_RGB[4][3] = {
    { 0, 4, 15 }, { 0, 15, 10 }, { 12, 0, 15 }, { 15, 4, 0 }
};

static void Fx_Level_Color(void)
{
    uint8_t l = (uint8_t)(level - 1);
    Fx_Breath(LVL_RGB[l][0], LVL_RGB[l][1], LVL_RGB[l][2], (uint16_t)(1400u - 250u * l));
}

static uint32_t Rand32(void)
{
    rngState = rngState * 1103515245u + 12345u;
    rngState ^= (msTicks & 0xFFu);
    return rngState >> 8;
}

static void Update_Frame_Ms(void)
{
    int32_t f = (int32_t)FRAME_MS[difficulty] - (int32_t)(level - 1) * FRAME_DEC;
    if (f < FRAME_MIN) f = FRAME_MIN;
    frameMs = (uint32_t)f;
}

static void Game_Reset(void)
{
    uint8_t i;
    score = 0;
    level = 1;
    birdY = (int16_t)(6 * FIX);
    birdV = 0;
    for (i = 0; i < MAX_PIPES; i++) pipes[i].active = 0;
    sinceSpawn = 0;
    nextSpacing = 5;
    Update_Frame_Ms();
    evS1 = evS2 = evS3 = 0;
    Show_Number(0);
    Leds_Level(1);
    Fx_Level_Color();
}

static void Score_Point(void)
{
    uint8_t newLevel;
    score++;
    Show_Number(score);

    newLevel = (uint8_t)(1 + score / LEVEL_SCORE);
    if (newLevel > MAX_LEVEL) newLevel = MAX_LEVEL;
    if (newLevel != level) {
        level = newLevel;
        Update_Frame_Ms();
        Leds_Level(level);
        Fx_Level_Color();
        Fx_Flash(15, 15, 6, 256, 1);                    /* เลเวลอัป : แฟลชเหลืองขาว ค้างนาน */
        SFX_PLAY(SFX_LEVEL, 3);
    } else {
        Fx_Flash(0, 15, 2, 256, 3);                     /* ได้คะแนน : แฟลชเขียว */
        Sfx_ScorePing(score);
    }
}

static void Spawn_Pipe(void)
{
    uint8_t i, type;
    for (i = 0; i < MAX_PIPES; i++) {
        if (!pipes[i].active) {
            if ((Rand32() % 100u) < MID_PCT[difficulty]) type = 2;
            else                                          type = (uint8_t)((Rand32() >> 3) & 1u);
            pipes[i].x = 15;
            pipes[i].type = type;
            pipes[i].scored = 0;
            pipes[i].active = 1;
            break;
        }
    }
    sinceSpawn = 0;
    nextSpacing = (uint8_t)(SPACING_MIN[difficulty] + (Rand32() % SPACING_RND[difficulty]));
}

static void Pipes_Shift(void)
{
    uint8_t i;
    for (i = 0; i < MAX_PIPES; i++) {
        if (!pipes[i].active) continue;
        pipes[i].x--;
        if (!pipes[i].scored && pipes[i].x < BIRD_COL) {
            pipes[i].scored = 1;
            Score_Point();
        }
        if (pipes[i].x < 0) pipes[i].active = 0;
    }
    sinceSpawn++;
    if (sinceSpawn >= nextSpacing) Spawn_Pipe();
}

static void Bird_Physics(void)
{
    const int16_t maxY = (int16_t)((WORLD_H - BIRD_H) * FIX);
    birdV += GRAVITY;
    if (birdV > MAX_FALL) birdV = MAX_FALL;
    birdY += birdV;
    if (birdY < 0)    { birdY = 0;    birdV = 0; }
    if (birdY > maxY) { birdY = maxY; birdV = 0; }
}

static uint8_t Check_Hit(void)
{
    uint8_t i, j;
    int16_t by = (int16_t)(birdY / FIX);
    int16_t a, b;
    for (i = 0; i < MAX_PIPES; i++) {
        if (!pipes[i].active || pipes[i].x != BIRD_COL) continue;
        for (j = 0; j < 2; j++) {
            a = PIPE_BLOCK[pipes[i].type][j][0];
            b = PIPE_BLOCK[pipes[i].type][j][1];
            if (b > a && by < b && (by + BIRD_H) > a) return 1;
        }
    }
    return 0;
}

static void Render_Game(void)
{
    uint8_t i, k, topUsed = 0, botUsed = 0;
    uint8_t topG[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t botG[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    int16_t by = (int16_t)(birdY / FIX), p;
    const uint8_t *spr = (birdV < 0) ? BIRD_B : BIRD_A;

    Lcd_BufClear();
    for (i = 0; i < MAX_PIPES; i++) {
        if (pipes[i].active && pipes[i].x >= 0 && pipes[i].x < 16) {
            lcdBuf[0][pipes[i].x] = PIPE_CELL[pipes[i].type][0];
            lcdBuf[1][pipes[i].x] = PIPE_CELL[pipes[i].type][1];
        }
    }
    for (k = 0; k < BIRD_H; k++) {
        p = (int16_t)(by + k);
        if (p < 8)               { topG[p] = spr[k];     topUsed = 1; }
        else if (p < WORLD_H)    { botG[p - 8] = spr[k]; botUsed = 1; }
    }
    if (topUsed) lcdBuf[0][BIRD_COL] = G_BIRD_TOP;
    if (botUsed) lcdBuf[1][BIRD_COL] = G_BIRD_BOT;

    Lcd_SetGlyph(G_BIRD_TOP, topG);
    Lcd_SetGlyph(G_BIRD_BOT, botG);
    Lcd_Flush();
}

static void Play_Game(void)
{
    uint32_t nextFrame;
    uint8_t frameCnt = 0;

    Game_Reset();
    Lcd_BufClear();
    Lcd_Flush();
    nextFrame = msTicks + frameMs;

    while (1) {
        if (evS1 || evS2 || evS3) {
            evS1 = evS2 = evS3 = 0;
            birdV = FLAP_V;
            Fx_Flash(6, 6, 8, 170, 7);                  /* กระพือปีก : แฟลชขาวสั้นๆ */
            SFX_PLAY(SFX_FLAP, 1);
        }

        Bird_Physics();
        if (Check_Hit()) break;

        if (++frameCnt >= PIPE_EVERY[difficulty]) {
            frameCnt = 0;
            Pipes_Shift();
            if (Check_Hit()) break;
        }

        Render_Game();

        while ((int32_t)(msTicks - nextFrame) < 0) { }
        nextFrame += frameMs;
        if ((int32_t)(msTicks - nextFrame) > 0) nextFrame = msTicks;
    }

    Fx_Solid(15, 0, 0);                                 /* ชน : แดง + แฟลชขาว */
    Fx_Flash(15, 15, 15, 256, 4);
    SFX_PLAY(SFX_HIT, 4);
    Render_Game();
    Delay_ms(500);
}

/* ================================== Screens ================================ */

static const char *const LV_NAME[3]  = { "EASY", "MID ", "HARD" };
static const char *const SEG_NAME[3] = { "EASy", "Mid ", "HArd" };

static void Show_Splash(void)
{
    if (!Sfx_IsActive()) Mute_Buzzer_HighZ();
    Leds_All(0);
    Lcd_Text("  FLAPPY  BIRD  ", "   EL334  Game  ");
    Show_Text4("rEdy");
    Fx_Rainbow(8);
    SFX_PLAY(SFX_START, 3);
    Delay_ms(1400);
}

/* วาดเมนูตามหน้า : 0 = MAIN, 1 = HI-SCORE, 2 = SOUND */
static void Menu_Draw(uint8_t page)
{
    uint8_t i, filled;

    Lcd_BufClear();
    if (page == 0) {
        Lcd_Puts(0, 0, "Lv:");
        Lcd_Puts(0, 3, LV_NAME[difficulty]);
        Lcd_Puts(0, 8, "Vol:");
        Lcd_Puts(0, 12, (volumeLevel == 0) ? "OFF" :
                        (volumeLevel == 1) ? "*  " :
                        (volumeLevel == 2) ? "** " : "***");
        Lcd_Puts(1, 0, "S1Lv S2Page S3Go");
        Show_Text4(SEG_NAME[difficulty]);
    } else if (page == 1) {
        Lcd_Puts(0, 0, "HI-SCORE Lv:");
        Lcd_Puts(0, 12, LV_NAME[difficulty]);
        Lcd_Puts(1, 0, "E");  Lcd_PutNum(1, 1, best[0], 3);
        Lcd_Puts(1, 5, "M");  Lcd_PutNum(1, 6, best[1], 3);
        Lcd_Puts(1, 10, "H"); Lcd_PutNum(1, 11, best[2], 3);
        Show_Number(best[difficulty]);
    } else {
        Lcd_Puts(0, 0, "SOUND  SW3+ SW2-");
        Lcd_Puts(1, 0, "Vol [");
        filled = (uint8_t)(((uint8_t)volumeLevel * 10u + 1u) / 3u);   /* 0,3,7,10 */
        for (i = 0; i < 10; i++) lcdBuf[1][5 + i] = (uint8_t)((i < filled) ? '#' : ' ');
        lcdBuf[1][15] = ']';
        Show_Text4("Snd0");
        dispBuf[3] = DIGIT_SEG[volumeLevel & 3];
    }
    Lcd_Flush();
    Leds_Level((uint8_t)(difficulty + 1));
}

/* เปลี่ยนความดังทีละขั้น (0..3) พร้อมเสียงตัวอย่างและแฟลชไฟ */
static void Volume_Change(int8_t delta, uint8_t page)
{
    int8_t v = (int8_t)((int8_t)volumeLevel + delta);
    if (v < 0) v = 0;
    if (v > 3) v = 3;
    if (v == (int8_t)volumeLevel) {                     /* สุดทางแล้ว -> แฟลชแดงเตือน */
        Fx_Flash(15, 0, 0, 200, 6);
        return;
    }
    volumeLevel = (uint8_t)v;
    SFX_PLAY(SFX_VOLP, 2);                              /* เสียงตัวอย่างที่ความดังใหม่ */
    Fx_Flash(15, 15, 0, 200, 4);
    Menu_Draw(page);
}

static void Menu_Run(void)
{
    uint8_t page = 0, up, dn, prevUp, prevDn;
    uint32_t lastVol = 0;

    Fx_Rainbow(3);                                      /* เมนู : ไฟสีรุ้งไหลช้าๆ */
    evS1 = evS2 = evS3 = 0;
    Menu_Draw(page);
    prevUp = (uint8_t)((PTA->PDIR >> OB_SW3_PIN) & 1u);
    prevDn = (uint8_t)((PTC->PDIR >> OB_SW2_PIN) & 1u);

    while (1) {
        if (evS3) {                                     /* S3 (Shield) : เริ่มเล่น */
            evS3 = 0;
            SFX_PLAY(SFX_START, 3);
            Fx_Flash(15, 15, 15, 256, 3);
            Delay_ms(350);
            break;
        }
        if (evS1) {                                     /* S1 : เปลี่ยนความยาก */
            evS1 = 0;
            difficulty = (uint8_t)((difficulty + 1) % 3);
            if (difficulty == 0)      { SFX_PLAY(SFX_LV_E, 2); Fx_Flash(0, 15, 3, 256, 3); }
            else if (difficulty == 1) { SFX_PLAY(SFX_LV_M, 2); Fx_Flash(15, 8, 0, 256, 3); }
            else                      { SFX_PLAY(SFX_LV_H, 2); Fx_Flash(15, 0, 0, 256, 3); }
            Menu_Draw(page);
        }
        if (evS2) {                                     /* S2 : เปลี่ยนหน้าเมนู */
            evS2 = 0;
            page = (uint8_t)((page + 1) % 3);
            SFX_PLAY(SFX_PAGE, 2);
            Fx_Flash(4, 4, 15, 200, 5);
            Menu_Draw(page);
        }

        /* ปุ่มบนบอร์ด FRDM : SW3 = เพิ่มความดัง, SW2 = ลดความดัง (ตรวจขอบขาลง) */
        up = (uint8_t)((PTA->PDIR >> OB_SW3_PIN) & 1u);
        dn = (uint8_t)((PTC->PDIR >> OB_SW2_PIN) & 1u);
        if ((msTicks - lastVol) > 120u) {
            if (prevUp && !up)      { Volume_Change(+1, page); lastVol = msTicks; }
            else if (prevDn && !dn) { Volume_Change(-1, page); lastVol = msTicks; }
        }
        prevUp = up;
        prevDn = dn;

        Delay_ms(20);
    }
    evS1 = evS2 = evS3 = 0;
}

static void Countdown_3s(void)
{
    uint8_t n, i;
    Leds_All(0);
    for (n = 3; n >= 1; n--) {
        for (i = 0; i < 4; i++) dispBuf[i] = DIGIT_SEG[n];
        Lcd_BufClear();
        Lcd_Puts(0, 3, "Get ready!");
        Lcd_PutNum(1, 7, n, 1);
        Lcd_Flush();

        if (n == 3)      { Fx_Solid(15, 0, 0); SFX_PLAY(SFX_BEEP3, 3); }   /* ไฟจราจร แดง -> เหลือง -> เขียว */
        else if (n == 2) { Fx_Solid(15, 7, 0); SFX_PLAY(SFX_BEEP2, 3); }
        else             { Fx_Solid(0, 15, 0); SFX_PLAY(SFX_BEEP1, 3); }
        Fx_Flash(6, 6, 6, 220, 4);
        Delay_ms(1000);
    }
    Fx_Flash(15, 15, 15, 256, 2);
    SFX_PLAY(SFX_GO, 3);
}

static void Game_Over_Screen(void)
{
    uint8_t newBest = 0;
    uint32_t start;

    if (score > best[difficulty]) { best[difficulty] = score; newBest = 1; }

    Lcd_BufClear();
    Lcd_Puts(0, 0, newBest ? "   NEW BEST!    " : "   GAME  OVER   ");
    Lcd_Puts(1, 0, "Score:");
    Lcd_PutNum(1, 6, score, 3);
    Lcd_Puts(1, 9, " B:");
    Lcd_PutNum(1, 12, best[difficulty], 3);
    Lcd_Flush();
    Show_Number(score);

    if (newBest) {
        SFX_PLAY(SFX_BEST, 5);
        Fx_Rainbow(24);                                 /* ทำสถิติใหม่ : รุ้งไหลเร็ว */
        Fx_Flash(15, 12, 0, 256, 1);
    } else {
        SFX_PLAY(SFX_OVER, 5);
        Fx_Breath(15, 0, 0, 1800);                      /* จบเกม : แดงหายใจช้า */
    }

    start = msTicks;
    if (newBest) {
        while ((msTicks - start) < 3000) Leds_All((uint8_t)((msTicks / 250) & 1));
    } else {                                            /* LED ดับลงทีละดวงตามเสียงที่ไล่ลง */
        while ((msTicks - start) < 3000) {
            uint32_t el = msTicks - start;
            uint8_t n = (uint8_t)((el >= 2000) ? 0 : (4 - el / 500));
            Leds_Level(n);
        }
    }
    Leds_All(0);
}

/* PTA4 (SW3) เป็นขา NMI ตามค่าเริ่มต้น : handler ว่างไว้กันโปรแกรมค้างเมื่อกดปุ่ม */
void NMI_Handler(void) { }

/* ================================== main =================================== */

int main(void)
{
    Init_Buzzer();          /* ตัด Buzzer เป็นอย่างแรก */
    Init_GPIO();
    Init_RGB();
    Init_ADC0();
    Init_PIT_Timer();
    Init_Switch_Interrupt();

    rngState ^= Read_ADC0();
    Lcd_Setup();
    Show_Splash();

    while (1) {
        Menu_Run();         /* เลือกความยาก / ดู High Score / ปรับเสียง */
        Countdown_3s();
        Play_Game();
        Game_Over_Screen(); /* แล้ววนกลับเมนู */
    }
}
