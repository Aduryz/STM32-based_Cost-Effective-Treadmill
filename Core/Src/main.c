/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===== SR1: 버튼 행 ===== */
#define ROW_SER_GPIO   GPIOB                    // SR1 직렬입력 포트
#define ROW_SER_PIN    GPIO_PIN_0               // SR1 직렬입력 핀
#define ROW_CLK_GPIO   GPIOA                    // SR1 클럭 포트
#define ROW_CLK_PIN    GPIO_PIN_1               // SR1 클럭 핀

/* Columns (입력 Pull-up, LOW=눌림) */
#define COL_GPIO       GPIOA                    // 컬럼 입력 포트
#define COL1_PIN       GPIO_PIN_2               // C1
#define COL2_PIN       GPIO_PIN_7               // C2
#define COL3_PIN       GPIO_PIN_6               // C3
#define COL4_PIN       GPIO_PIN_5               // C4

/* ===== SR2: 7-세그 ===== */
#define SR2_SER_GPIO   GPIOB                    // SR2 직렬입력 포트
#define SR2_SER_PIN    GPIO_PIN_1               // SR2 직렬입력 핀
#define SR2_CLK_GPIO   GPIOB                    // SR2 클럭 포트
#define SR2_CLK_PIN    GPIO_PIN_4               // SR2 클럭 핀
#define SR2_CLR_GPIO   GPIOB                    // SR2 클리어 포트(Active-Low)
#define SR2_CLR_PIN    GPIO_PIN_5               // SR2 클리어 핀

/* ===== SR3: LED/모터/부저 ===== */
#define SR3_SER_GPIO   GPIOA                    // SR3 직렬입력 포트
#define SR3_SER_PIN    GPIO_PIN_3               // SR3 직렬입력 핀
#define SR3_CLK_GPIO   GPIOA                    // SR3 클럭 포트
#define SR3_CLK_PIN    GPIO_PIN_4               // SR3 클럭 핀

/* ===== LCD Soft I2C (PB6=SCL, PB7=SDA) ===== */
#define LCD_SCL_GPIO   GPIOB                    // LCD SCL 포트
#define LCD_SCL_PIN    GPIO_PIN_6               // LCD SCL 핀 (OD)
#define LCD_SDA_GPIO   GPIOB                    // LCD SDA 포트
#define LCD_SDA_PIN    GPIO_PIN_7               // LCD SDA 핀 (OD)
#define LCD_I2C_ADDR_7BIT  0x27                 // PCF8574 기본 7비트 주소
#define LCD_I2C_ADDR_WRITE ((LCD_I2C_ADDR_7BIT<<1) | 0x00) // 쓰기 어드레스(8비트)

/* ===== SR3 비트 ===== */
#define SR3_BIT_BUZZ   0                        // 부저 비트
#define SR3_BIT_LED_R  1                        // LED 빨강
#define SR3_BIT_LED_Y  2                        // LED 노랑
#define SR3_BIT_LED_G  3                        // LED 초록
#define SR3_BIT_IN1    4                        // 모터 IN1 (팬)
#define SR3_BIT_IN2    5                        // 모터 IN2 (팬)
#define SR3_BIT_IN3    6                        // 모터 IN3 (러닝)
#define SR3_BIT_IN4    7                        // 모터 IN4 (러닝)

/* ===== 7-세그먼트 패턴(공통 GND, Active-HIGH). ===== */
static inline uint8_t seg_digit_pattern(int d){ // 숫자→세그먼트 매핑
  switch(d){
    case 0: return 0x7E;        // 0 표시 패턴
    case 1: return 0x30;        // 1 표시 패턴
    case 2: return 0x6D;        // 2 표시 패턴
    case 3: return 0x79;        // 3 표시 패턴
    default: return 0x00;       // 공백(끄기)
  }
}

/* ===== 팬/러닝 속도(%) ===== */
#define FAN_S1_PCT   50          // 선풍기 S1 듀티(%)
#define FAN_S2_PCT   75          // 선풍기 S2 듀티(%)
#define FAN_S3_PCT   90          // 선풍기 S3 듀티(%)
#define TM_S1_PCT    50          // 러닝머신 S1 듀티(%)
#define TM_S2_PCT    60          // 러닝머신 S2 듀티(%)
#define TM_S3_PCT    70          // 러닝머신 S3 듀티(%)

/* ===== 스캔/디바운스 파라미터 (비블로킹) ===== */
#define SCAN_SETTLE_US    100     // 행 선택 후 안정 대기(us)
#define DEBOUNCE_MS         8     // 디바운스 시간(ms)

/* ===== TIM1: 10 kHz ===== */
#define PWM_TIM_PRESCALER   0     // PSC=0
#define PWM_TIM_PERIOD      399   // ARR=399 → (4MHz)/(1*(399+1))=10kHz
#define PWM_CH1_PORT        GPIOA  // TIM1_CH1 포트
#define PWM_CH1_PIN         GPIO_PIN_8 // TIM1_CH1 핀(팬 EN)
#define PWM_CH1_AF          GPIO_AF1_TIM1 // TIM1_CH1 대체기능
#define PWM_CH4_PORT        GPIOA  // TIM1_CH4 포트
#define PWM_CH4_PIN         GPIO_PIN_11 // TIM1_CH4 핀(러닝 EN)
#define PWM_CH4_AF          GPIO_AF1_TIM1 // TIM1_CH4 대체기능

/* ===== 램프/킥 ===== */
#define RAMP_STEP_PCT       4     // 램프 단계(%)
#define RAMP_STEP_DELAY_MS  8     // 단계 대기(ms)
#define KICK_THRESH_PCT     35    // 팬 킥 임계(%)
#define KICK_PCT            90    // 팬 킥 듀티(%)
#define KICK_MS             140   // 팬 킥 시간(ms)
#define TM_KICK_THRESH_PCT  60    // 러닝 킥 임계(%)
#define TM_KICK_PCT         100   // 러닝 킥 듀티(%)
#define TM_KICK_MS          250   // 러닝 킥 시간(ms)

/* ===== 리프레시 ===== */
#define SEG_REFRESH_MS      25    // 7-세그먼트 재도장 주기(ms)
#define SR3_REFRESH_MS      10    // SR3 재적용 주기(ms)

/* ===== LCD/HD44780: PCF8574 비트 매핑 ===== */
#define LCD_RS   (1U<<0)          // RS
#define LCD_RW   (1U<<1)          // RW
#define LCD_EN   (1U<<2)          // EN
#define LCD_BL   (1U<<3)          // 백라이트
#define LCD_D4   (1U<<4)          // D4
#define LCD_D5   (1U<<5)          // D5
#define LCD_D6   (1U<<6)          // D6
#define LCD_D7   (1U<<7)          // D7

/* ===== 타이머/스톱워치 모드 ===== */
typedef enum { MODE_STOPWATCH=0, MODE_SET_TIMER, MODE_RUN_TIMER, MODE_ALARM } OperatingMode; // 동작 모드 열거형
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim1;                  // TIM1 핸들 (PWM)
UART_HandleTypeDef huart1;                // USART1 핸들 (HC-05)

static uint8_t rxData;                    // UART 수신 바이트 버퍼
static volatile uint8_t uart_rx_flag = 0; // UART 수신 플래그
static volatile uint8_t received_char = 0;// 마지막 수신 문자

static uint8_t  seg_cur_digit = 0;        // 현재 7-세그먼트 표시 숫자
static uint8_t  sr3_state = 0x00;         // SR3(LED/부저/모터) 출력 래치 상태

static uint8_t  fan_pct = 0;              // 선풍기 듀티(%)
static uint8_t  tm_pct  = 0;              // 러닝 듀티(%)

static OperatingMode g_mode = MODE_STOPWATCH; // 현재 동작 모드
static uint8_t  g_run = 0;                // 실행 플래그
static uint32_t g_sw_ms = 0;              // 스톱워치 누적(ms)
static uint32_t g_timer_ms = 0;           // 타이머 남은(ms)
static uint32_t g_last_tick_ms = 0;       // 시간 적분 기준(ms)
static uint8_t  g_alarm_on = 0;           // 알람 동작 여부
static uint32_t g_buzz_last_ms = 0;       // 부저 토글 타이밍
static uint8_t  g_buzz_state = 0;         // 부저 현재 상태(토글)

/* LCD 더티 플래그/스냅샷(변경시에만 갱신) */
static uint8_t  lcd_dirty = 1;            // LCD 재갱신 필요 플래그
static uint32_t lcd_last_ms = 0;          // LCD 갱신 타이밍
static OperatingMode lcd_prev_mode;       // 이전 모드
static uint8_t lcd_prev_run;              // 이전 실행 플래그
static uint32_t lcd_prev_sw_ms;           // 이전 스톱워치(ms)
static uint32_t lcd_prev_timer_ms;        // 이전 타이머(ms)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* 내부 유틸/로직 프로토타입 */
static inline void delay_us_soft(uint32_t us);         // 소프트 us 지연
static inline void sr_pulse_slow(GPIO_TypeDef* port, uint16_t pin); // SR 클럭 펄스
static inline void sr_push_msb(GPIO_TypeDef* ser_port, uint16_t ser_pin,
                               GPIO_TypeDef* clk_port, uint16_t clk_pin,
                               uint8_t v);            // 8비트 시프트(MSB-first)

static inline void sr3_apply(void);                   // SR3에 상태 적용
static inline void sr3_set_bit(uint8_t bit, uint8_t on); // SR3 비트 세팅
static inline void sr3_set_led_count(int n);          // LED 막대기 표시

static inline void seg_show_digit_hard(uint8_t d);    // 7-세그먼트 하드 재도장
static inline void seg_set_digit(uint8_t d);          // 7-세그먼트 현재값 변경

static inline uint8_t row_pattern_low_active(uint8_t row_0to3); // SR1 행 패턴
static inline void sr0_push(uint8_t v);               // SR1로 바이트 출력
static inline uint8_t read_cols(void);                // 컬럼 읽기
static int scan_buttons_once(void);                   // 버튼 한 번 스캔
static int get_new_press(void);                       // 디바운스+엣지검출

static inline void fan_set_percent(uint8_t pct);      // 팬 듀티 설정
static void fan_ramp_to(uint8_t target);              // 팬 램프
static inline void fan_drive_dir_fwd(void);           // 팬 정회전 IN 세팅
static void fan_apply_with_kick(uint8_t target);      // 팬 킥+램프 적용
static void fan_stop_coast(void);                     // 팬 정지(coast)

static inline void tm_set_percent(uint8_t pct);       // 러닝 듀티 설정
static void tm_ramp_to(uint8_t target);               // 러닝 램프
static inline void tm_drive_dir_fwd(void);            // 러닝 정회전 IN 세팅
static void tm_apply_with_kick(uint8_t target);       // 러닝 킥+램프 적용
static void tm_stop_coast(void);                      // 러닝 정지(coast)

static inline void SDA_H(void);                       // SDA High
static inline void SDA_L(void);                       // SDA Low
static inline void SCL_H(void);                       // SCL High
static inline void SCL_L(void);                       // SCL Low
static void i2c_start(void);                          // I2C START
static void i2c_stop(void);                           // I2C STOP
static void i2c_write_byte(uint8_t data);             // I2C 바이트 전송
static inline void pcf8574_write(uint8_t v);          // PCF8574 한 바이트 쓰기

static void lcd_send_nibble(uint8_t nib, uint8_t rs); // LCD 4비트 전송
static void lcd_send_byte(uint8_t b, uint8_t rs);     // LCD 8비트 전송
static void lcd_cmd(uint8_t c);                       // LCD 명령
static void lcd_data(uint8_t d);                      // LCD 데이터
static void lcd_init(void);                           // LCD 초기화
static void lcd_set_cursor(uint8_t col, uint8_t row); // LCD 커서
static void lcd_print(const char* s);                 // 문자열 출력
static void lcd_print_mmss(uint32_t secs);            // MM:SS 포맷 출력
static void lcd_print_hhmmss(uint32_t ms);            // HH:MM:SS 포맷 출력

static void periodic_refresh_and_time(void);          // 주기적 유지/시간/LCD
static void on_press_S9(void);                        // 키 S9 핸들러
static void on_press_S10(void);                       // 키 S10 핸들러
static void on_press_S11(void);                       // 키 S11 핸들러
static void on_press_S12(void);                       // 키 S12 핸들러
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ===== 소프트 지연/시프트 공통 ===== */
static inline void delay_us_soft(uint32_t us){ for(uint32_t i=0;i<us*6;i++){ __NOP(); } } // 4MHz에서 대충 1us 근처 지연
static inline void sr_pulse_slow(GPIO_TypeDef* port, uint16_t pin){
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); delay_us_soft(2); // 클럭 High 짧게 유지
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET); delay_us_soft(2); // 클럭 Low
}
static inline void sr_push_msb(GPIO_TypeDef* ser_port, uint16_t ser_pin,
                               GPIO_TypeDef* clk_port, uint16_t clk_pin,
                               uint8_t v){
  for(int i=0;i<8;i++){                                 // 8비트 전송
    uint8_t b = (v & 0x80U) ? 1U : 0U;                  // 최상위비트부터
    HAL_GPIO_WritePin(ser_port, ser_pin, b?GPIO_PIN_SET:GPIO_PIN_RESET); // SER 세팅
    sr_pulse_slow(clk_port, clk_pin);                   // CLK 펄스
    v <<= 1;                                            // 다음 비트로 이동
  }
}

/* ===== SR3(LED/모터/부저) ===== */
static inline void sr3_apply(void){ sr_push_msb(SR3_SER_GPIO, SR3_SER_PIN, SR3_CLK_GPIO, SR3_CLK_PIN, sr3_state); } // 현재 sr3_state를 실제로 출력
static inline void sr3_set_bit(uint8_t bit, uint8_t on){
  if(on) sr3_state |=  (1U<<bit); else sr3_state &= ~(1U<<bit); // 비트 조작
  sr3_apply();                                                  // 즉시 적용
}
static inline void sr3_set_led_count(int n){                    // 3색 LED를 단계막대로 표시
  sr3_set_bit(SR3_BIT_LED_R, (n>=1));
  sr3_set_bit(SR3_BIT_LED_Y, (n>=2));
  sr3_set_bit(SR3_BIT_LED_G, (n>=3));
}

/* ===== 7-세그먼트 ===== */
static inline void seg_show_digit_hard(uint8_t d){
  uint8_t patt = seg_digit_pattern(d);                          // 숫자→패턴
  HAL_GPIO_WritePin(SR2_CLR_GPIO, SR2_CLR_PIN, GPIO_PIN_RESET); // CLR=Low로 클리어
  for(volatile int i=0;i<120;i++) __NOP();                      // 아주 짧은 펄스 유지
  HAL_GPIO_WritePin(SR2_CLR_GPIO, SR2_CLR_PIN, GPIO_PIN_SET);   // CLR 비활성화(High)
  for(volatile int i=0;i<120;i++) __NOP();                      // 안정 대기
  sr_push_msb(SR2_SER_GPIO, SR2_SER_PIN, SR2_CLK_GPIO, SR2_CLK_PIN, patt); // 패턴 전송1
  sr_push_msb(SR2_SER_GPIO, SR2_SER_PIN, SR2_CLK_GPIO, SR2_CLK_PIN, patt); // 패턴 전송2(재도장)
}
static inline void seg_set_digit(uint8_t d){ seg_cur_digit = d; seg_show_digit_hard(d); } // 상태 반영+표시

/* ===== SR1 ===== */
static inline uint8_t row_pattern_low_active(uint8_t row_0to3){
  switch(row_0to3){ case 0: return 0xF7; case 1: return 0xFB; case 2: return 0xFD; default:return 0xFE; } // 해당 행만 Low 활성
}
static inline void sr0_push(uint8_t v){ sr_push_msb(ROW_SER_GPIO, ROW_SER_PIN, ROW_CLK_GPIO, ROW_CLK_PIN, v); } // SR1로 출력
static inline uint8_t read_cols(void){
  uint8_t v=0;
  v |= HAL_GPIO_ReadPin(COL_GPIO, COL1_PIN) ? 0x01 : 0x00; // 컬럼1
  v |= HAL_GPIO_ReadPin(COL_GPIO, COL2_PIN) ? 0x02 : 0x00; // 컬럼2
  v |= HAL_GPIO_ReadPin(COL_GPIO, COL3_PIN) ? 0x04 : 0x00; // 컬럼3
  v |= HAL_GPIO_ReadPin(COL_GPIO, COL4_PIN) ? 0x08 : 0x00; // 컬럼4
  return v;                                                // 하위4비트 유효
}
static int scan_buttons_once(void){
  for(int r=0;r<4;r++){                                    // 행 0~3 순회
    sr0_push( row_pattern_low_active(r) );                 // 해당 행만 잠깐 Low
    for(volatile int i=0;i<(SCAN_SETTLE_US); i++) __NOP(); // 정착 대기
    uint8_t cols = read_cols();                            // 컬럼 샘플링
    sr0_push(0x00);                                        // 즉시 비선택 복귀(All Low)
    if((cols & 0x0F) != 0x0F){                             // 하나라도 눌림(LOW) 있으면
      for(int c=0;c<4;c++){                                // 컬럼별 검사
        if(!(cols & (1<<c))){ return r*4 + c + 1; }        // 버튼 ID 1~12 반환
      }
    }
  }
  return 0;                                                // 없으면 0
}
/* 시간 기반 디바운스 + 엣지 검출 */
static int get_new_press(void){
  static int last_phys = 0;                                // 직전 물리 상태
  static int stable_val = 0;                               // 안정 상태
  static uint32_t last_change_ms = 0;                      // 마지막 변화 시각
  uint32_t now = HAL_GetTick();                            // 현재 ms
  int cur = scan_buttons_once();                           // 현재 입력
  if(cur != last_phys){ last_phys = cur; last_change_ms = now; } // 변화 감지
  if((now - last_change_ms) >= DEBOUNCE_MS){               // 디바운스 통과
    if(stable_val != last_phys){                           // 안정 상태 갱신
      stable_val = last_phys;
      if(stable_val != 0){ return stable_val; }            // 눌림 엣지 리턴
    }
  }
  return 0;                                                // 변화 없음
}

/* ===== 팬 PWM (CH1) ===== */
static inline void fan_set_percent(uint8_t pct){
  if(pct>100) pct=100;                                     // 0~100 제한
  uint32_t ccr = (uint32_t)((PWM_TIM_PERIOD + 1U) * pct / 100U); // 듀티→CCR
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);       // CH1 비교값 설정
  fan_pct = pct;                                           // 상태 저장
}
static void fan_ramp_to(uint8_t target){
  uint8_t cur=fan_pct;                                     // 현재 듀티
  if(cur==target){ fan_set_percent(target); return; }      // 동일하면 종료
  if(cur<target){                                          // 상승 램프
    for(uint8_t p=cur; p<target; ){
      uint8_t next=(uint8_t)((p+RAMP_STEP_PCT>target)?target:p+RAMP_STEP_PCT); // 다음 단계
      fan_set_percent(next); HAL_Delay(RAMP_STEP_DELAY_MS); p=next;            // 적용+대기
    }
  }
  else{                                                   // 하강 램프
    for(uint8_t p=cur; p>target; ){
      uint8_t next=(p>RAMP_STEP_PCT)?(uint8_t)(p-RAMP_STEP_PCT):target;        // 다음 단계
      if(next<target) next=target;                           // 하한 보정
      fan_set_percent(next); HAL_Delay(RAMP_STEP_DELAY_MS); p=next;            // 적용+대기
      if(p==target) break;                                   // 완료
    }
  }
  fan_set_percent(target);                                 // 최종 보정
}
static inline void fan_drive_dir_fwd(void){
  sr3_state &= ~((1U<<SR3_BIT_IN2));                       // IN2=0
  sr3_state |=  (1U<<SR3_BIT_IN1);                         // IN1=1 (정회전)
  sr3_apply();                                             // 상태 적용
}
static void fan_apply_with_kick(uint8_t target){
  fan_drive_dir_fwd();                                     // 방향 세팅
  if(fan_pct==0 && target>0 && target<=KICK_THRESH_PCT){   // 저듀티 시 시동 킥
    fan_set_percent(KICK_PCT); HAL_Delay(KICK_MS);         // 킥 구동
  }
  fan_ramp_to(target);                                     // 목표까지 램프
}
static void fan_stop_coast(void){
  fan_ramp_to(0);                                          // 듀티 0으로
  sr3_state &= ~((1U<<SR3_BIT_IN1)|(1U<<SR3_BIT_IN2));     // IN1/IN2 둘 다 0
  sr3_apply();                                             // 적용
}

/* ===== 러닝머신 PWM (CH4) ===== */
static inline void tm_set_percent(uint8_t pct){
  if(pct>100) pct=100;                                     // 0~100 제한
  uint32_t ccr = (uint32_t)((PWM_TIM_PERIOD + 1U) * pct / 100U); // 듀티→CCR
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr);       // CH4 비교값 설정
  tm_pct = pct;                                            // 상태 저장
}
static void tm_ramp_to(uint8_t target){
  uint8_t cur=tm_pct;                                      // 현재 듀티
  if(cur==target){ tm_set_percent(target); return; }       // 동일하면 종료
  if(cur<target){                                          // 상승 램프
    for(uint8_t p=cur; p<target; ){
      uint8_t next=(uint8_t)((p+RAMP_STEP_PCT>target)?target:p+RAMP_STEP_PCT);
      tm_set_percent(next); HAL_Delay(RAMP_STEP_DELAY_MS); p=next;
    }
  }
  else{                                                   // 하강 램프
    for(uint8_t p=cur; p>target; ){
      uint8_t next=(p>RAMP_STEP_PCT)?(uint8_t)(p-RAMP_STEP_PCT):target;
      if(next<target) next=target;
      tm_set_percent(next); HAL_Delay(RAMP_STEP_DELAY_MS); p=next; if(p==target) break;
    }
  }
  tm_set_percent(target);                                  // 최종 보정
}
static inline void tm_drive_dir_fwd(void){
  sr3_state &= ~((1U<<SR3_BIT_IN4));                       // IN4=0
  sr3_state |=  (1U<<SR3_BIT_IN3);                         // IN3=1 (정회전)
  sr3_apply();                                             // 적용
}
static void tm_apply_with_kick(uint8_t target){
  tm_drive_dir_fwd();                                      // 방향 세팅
  if(tm_pct==0 && target>0 && target<=TM_KICK_THRESH_PCT){ // 저듀티 시 시동 킥
    tm_set_percent(TM_KICK_PCT); HAL_Delay(TM_KICK_MS);    // 킥 구동
  }
  tm_ramp_to(target);                                      // 목표까지 램프
}
static void tm_stop_coast(void){
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);         // 듀티 0
  tm_pct = 0;                                              // 상태 갱신
  sr3_state &= ~((1U<<SR3_BIT_IN3)|(1U<<SR3_BIT_IN4));     // IN3/IN4 둘 다 0
  sr3_apply();                                             // 적용
}

/* ========================= LCD (소프트 I2C + HD44780) ========================= */
/* GPIO 토글 헬퍼 */
static inline void SDA_H(void){ HAL_GPIO_WritePin(LCD_SDA_GPIO, LCD_SDA_PIN, GPIO_PIN_SET); } // SDA=High
static inline void SDA_L(void){ HAL_GPIO_WritePin(LCD_SDA_GPIO, LCD_SDA_PIN, GPIO_PIN_RESET);} // SDA=Low
static inline void SCL_H(void){ HAL_GPIO_WritePin(LCD_SCL_GPIO, LCD_SCL_PIN, GPIO_PIN_SET); } // SCL=High
static inline void SCL_L(void){ HAL_GPIO_WritePin(LCD_SCL_GPIO, LCD_SCL_PIN, GPIO_PIN_RESET);} // SCL=Low

/* START/STOP/WriteByte(bit-bang) */
static void i2c_start(void){ SDA_H(); SCL_H(); delay_us_soft(3); SDA_L(); delay_us_soft(3); SCL_L(); } // START
static void i2c_stop(void){ SDA_L(); SCL_H(); delay_us_soft(3); SDA_H(); }                              // STOP
static void i2c_write_byte(uint8_t data){
  for(int i=7;i>=0;i--){                                     // MSB-first
    (data & (1U<<i)) ? SDA_H() : SDA_L();                    // 데이터 비트
    SCL_H(); delay_us_soft(2); SCL_L(); delay_us_soft(1);    // 클럭 토글
  }
  SDA_H(); SCL_H(); delay_us_soft(2); SCL_L();               // ACK 무시(라인 해제)
}
static inline void pcf8574_write(uint8_t v){ i2c_start(); i2c_write_byte(LCD_I2C_ADDR_WRITE); i2c_write_byte(v); i2c_stop(); } // 1바이트 전송

/* LCD 4비트/8비트 전송 */
static void lcd_send_nibble(uint8_t nib, uint8_t rs){
  uint8_t out = (rs?LCD_RS:0) | LCD_BL;                      // RS/백라이트 설정
  if(nib & 0x1) out |= LCD_D4;                               // 하위 비트→D4
  if(nib & 0x2) out |= LCD_D5;                               // →D5
  if(nib & 0x4) out |= LCD_D6;                               // →D6
  if(nib & 0x8) out |= LCD_D7;                               // →D7
  pcf8574_write(out);                                        // 데이터 세팅
  pcf8574_write(out | LCD_EN);                               // EN High(래치)
  delay_us_soft(30);                                         // 펄스 폭
  pcf8574_write(out);                                        // EN Low
}
static void lcd_send_byte(uint8_t b, uint8_t rs){ lcd_send_nibble((b>>4)&0x0F, rs); lcd_send_nibble(b&0x0F, rs); if(!rs) HAL_Delay(1); } // 명령이면 약간 대기
static void lcd_cmd(uint8_t c){ lcd_send_byte(c, 0); }       // 명령 전송
static void lcd_data(uint8_t d){ lcd_send_byte(d, 1); }      // 데이터 전송
static void lcd_init(void){
  HAL_Delay(30);                                             // 전원 안정 대기
  lcd_send_nibble(0x03,0); HAL_Delay(4);                     // 8비트 모드 3번
  lcd_send_nibble(0x03,0); HAL_Delay(4);
  lcd_send_nibble(0x03,0); HAL_Delay(4);
  lcd_send_nibble(0x02,0); HAL_Delay(2);                     // 4비트 모드 진입
  lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x01); HAL_Delay(2); lcd_cmd(0x06); // 2라인/표시ON/클리어/증가모드
}
static void lcd_set_cursor(uint8_t col, uint8_t row){
  static const uint8_t offsets[] = {0x00,0x40,0x14,0x54};    // 줄 오프셋
  lcd_cmd(0x80 | (offsets[row] + col));                      // DDRAM 주소 설정
}
static void lcd_print(const char* s){ while(*s){ lcd_data((uint8_t)*s++); } } // 문자열 연속 출력
static void lcd_print_mmss(uint32_t secs){
  uint32_t mm = secs/60U, ss = secs%60U;                     // 분/초 분리
  char buf[6]; buf[0]='0'+(mm/10)%10; buf[1]='0'+(mm%10); buf[2]=':'; buf[3]='0'+(ss/10); buf[4]='0'+(ss%10); buf[5]=0; // MM:SS
  lcd_print(buf);                                            // 출력
}
static void lcd_print_hhmmss(uint32_t ms){
  uint32_t s = ms/1000U, m = s/60U, h = m/60U;               // 시/분/초
  char buf[9];
  buf[0]='0'+(h/10)%10; buf[1]='0'+(h%10); buf[2]=':';       // HH:
  buf[3]='0'+((m%60)/10); buf[4]='0'+((m%60)%10); buf[5]=':';// MM:
  buf[6]='0'+((s%60)/10); buf[7]='0'+((s%60)%10); buf[8]=0;  // SS
  lcd_print(buf);                                            // 출력
}

/* ========= 주기적 리프레시 & 시간/부저 & LCD(더티 갱신) ========= */
static void periodic_refresh_and_time(void){
  static uint32_t last_seg_ms = 0;                           // 7-세그먼트 타이머
  static uint32_t last_sr3_ms = 0;                           // SR3 유지 타이머
  uint32_t now = HAL_GetTick();                              // 현재 ms

  if(now - last_seg_ms >= SEG_REFRESH_MS){                   // 주기 도래시
    last_seg_ms = now; seg_show_digit_hard(seg_cur_digit);   // 재도장
  }

  if(now - last_sr3_ms >= SR3_REFRESH_MS){                   // SR3 keep-alive
    last_sr3_ms = now;
    if(fan_pct>0){ __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)((PWM_TIM_PERIOD+1U)*fan_pct/100U)); fan_drive_dir_fwd(); } // 팬 유지
    if(tm_pct >0){ __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)((PWM_TIM_PERIOD+1U)*tm_pct /100U)); tm_drive_dir_fwd(); }  // 러닝 유지
    sr3_apply();                                             // 출력 재적용
  }

  if(g_last_tick_ms == 0) g_last_tick_ms = now;              // 첫 기준 설정
  uint32_t dt = now - g_last_tick_ms;                        // 경과
  if(dt){ g_last_tick_ms = now;                              // 기준 갱신
    if(g_mode==MODE_STOPWATCH && g_run){ g_sw_ms += dt; }    // 스톱워치
    if(g_mode==MODE_RUN_TIMER && g_run){                     // 타이머 감소
      if(g_timer_ms > dt) g_timer_ms -= dt;
      else { g_timer_ms = 0; g_run = 0; g_mode = MODE_ALARM; g_alarm_on = 1; lcd_dirty = 1; } // 타임업
    }
  }

  /* 부저 토글 (알람 중에만) */
  if(g_alarm_on){
    if(now - g_buzz_last_ms >= 1){ g_buzz_last_ms = now; g_buzz_state ^= 1; sr3_set_bit(SR3_BIT_BUZZ, g_buzz_state); } // 1ms 토글
  }
  else if(g_buzz_state){ g_buzz_state = 0; sr3_set_bit(SR3_BIT_BUZZ, 0); } // 알람 종료시 끄기

  /* LCD: 150ms 주기, 내용 변경시에만 갱신 */
  if(now - lcd_last_ms >= 150){
    lcd_last_ms = now;
    if(lcd_dirty ||
       lcd_prev_mode    != g_mode ||
       lcd_prev_run     != g_run  ||
       (g_mode==MODE_STOPWATCH && (g_sw_ms/1000U)!=(lcd_prev_sw_ms/1000U)) ||
       ((g_mode==MODE_SET_TIMER || g_mode==MODE_RUN_TIMER) && (g_timer_ms/1000U)!=(lcd_prev_timer_ms/1000U))
      ){
      lcd_prev_mode = g_mode; lcd_prev_run = g_run;          // 스냅샷 갱신
      lcd_prev_sw_ms = g_sw_ms; lcd_prev_timer_ms = g_timer_ms;
      lcd_dirty = 0;                                        // 더티 해제

      if(g_mode==MODE_STOPWATCH){                           // 모드별 LCD
        lcd_set_cursor(0,0); lcd_print("STOPWATCH       ");
        lcd_set_cursor(0,1); lcd_print("TIME "); lcd_print_hhmmss(g_sw_ms); lcd_print("   ");
      }
      else if(g_mode==MODE_SET_TIMER){
        uint32_t secs = g_timer_ms/1000U;
        lcd_set_cursor(0,0); lcd_print("SET TIMER       ");
        lcd_set_cursor(0,1); lcd_print("TIME "); lcd_print_mmss(secs); lcd_print("   ");
      }
      else if(g_mode==MODE_RUN_TIMER){
        uint32_t secs = g_timer_ms/1000U;
        lcd_set_cursor(0,0); lcd_print(g_run? "TIMER RUN       ":"TIMER PAUSE     ");
        lcd_set_cursor(0,1); lcd_print("LEFT "); lcd_print_mmss(secs); lcd_print("   ");
      }
      else{ // MODE_ALARM
        lcd_set_cursor(0,0); lcd_print("!!! TIME UP !!! ");
        lcd_set_cursor(0,1); lcd_print("PRESS S9/S10    ");
      }
    }
  }
}

/* ========= UART1 인터럽트 콜백 ========= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART1){                             // USART1 수신
    received_char = rxData;                                  // 바이트 저장
    uart_rx_flag = 1;                                        // 플래그 세트
    HAL_UART_Receive_IT(&huart1, &rxData, 1);                // 다음 바이트 수신 재개
  }
}

/* ========= 모드/키 처리 ========= */
static void on_press_S9(void){
  if(g_mode==MODE_STOPWATCH){ g_run = !g_run; lcd_dirty = 1; } // 스톱워치 시작/일시정지
  else if(g_mode==MODE_SET_TIMER){ if(g_timer_ms>0){ g_run=1; g_mode=MODE_RUN_TIMER; lcd_dirty=1; } } // 타이머 시작
  else if(g_mode==MODE_RUN_TIMER){ g_run = !g_run; lcd_dirty=1; } // 타이머 일시정지/재개
  else if(g_mode==MODE_ALARM){ g_alarm_on = 0; g_mode = MODE_SET_TIMER; lcd_dirty=1; } // 알람 해제→설정모드
}
static void on_press_S10(void){
  if(g_mode==MODE_STOPWATCH){ g_run=0; g_sw_ms=0; g_mode=MODE_SET_TIMER; } // 스톱워치 리셋→타이머설정
  else { g_alarm_on=0; g_run=0; g_mode=MODE_STOPWATCH; }                   // 타이머계열→스톱워치
  lcd_dirty=1;                                                             // LCD 갱신
}
static void on_press_S11(void){ // +30s
  if(g_mode==MODE_SET_TIMER || g_mode==MODE_RUN_TIMER || g_mode==MODE_ALARM){
    if(g_mode==MODE_ALARM){ g_alarm_on=0; g_mode=MODE_SET_TIMER; } // 알람 중 누르면 해제 후 설정
    g_timer_ms += 30000; lcd_dirty=1;                              // 30초 증가
  }
}
static void on_press_S12(void){ // -30s
  if(g_mode==MODE_SET_TIMER || g_mode==MODE_RUN_TIMER || g_mode==MODE_ALARM){
    if(g_mode==MODE_ALARM){ g_alarm_on=0; g_mode=MODE_SET_TIMER; } // 알람 해제
    if(g_timer_ms >= 30000) g_timer_ms -= 30000; else g_timer_ms = 0; // 30초 감소(하한 0)
    lcd_dirty=1;                                                    // LCD 갱신
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(SR2_CLR_GPIO, SR2_CLR_PIN, GPIO_PIN_SET); // 7-세그먼트 CLR=High

  lcd_init();                                                // LCD 초기화(소프트 I2C)
  lcd_set_cursor(0,0); lcd_print("STOPWATCH       ");        // 부팅 메시지 1행
  lcd_set_cursor(0,1); lcd_print("TIME 00:00:00   ");        // 부팅 메시지 2행

  sr3_state = 0x00; sr3_apply();                             // SR3(부저/모터/LED) 모두 Off
  seg_set_digit(0);                                          // 7-세그먼트 0 표시

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                  // TIM1 CH1 PWM 시작(팬 EN)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);                  // TIM1 CH4 PWM 시작(러닝 EN)
  fan_set_percent(0); tm_set_percent(0);                     // 초기 듀티 0
  __HAL_TIM_MOE_ENABLE(&htim1);                              // 고급타이머 메인 출력 Enable

  HAL_UART_Receive_IT(&huart1, &rxData, 1);                  // UART 인터럽트 수신 시작

  sr3_set_led_count(1); HAL_Delay(80);                       // 부팅 LED 스윕 1
  sr3_set_led_count(2); HAL_Delay(80);                       // 부팅 LED 스윕 2
  sr3_set_led_count(3); HAL_Delay(80);                       // 부팅 LED 스윕 3
  sr3_set_led_count(0); HAL_Delay(60);                       // 끄기
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    periodic_refresh_and_time();                            // 주기 리프레시/시간/LCD/부저 관리

    int b = get_new_press();                                // 새 버튼 엣지 읽기(없으면 0)
    if(b){                                                  // 눌림이 있으면 분기
      if (b==1){ sr3_set_led_count(1); fan_apply_with_kick(FAN_S1_PCT); }       // 팬 1단
      else if (b==2){ sr3_set_led_count(2); fan_apply_with_kick(FAN_S2_PCT); }  // 팬 2단
      else if (b==3){ sr3_set_led_count(3); fan_apply_with_kick(FAN_S3_PCT); }  // 팬 3단
      else if (b==4){ sr3_set_led_count(0); fan_stop_coast(); }                 // 팬 OFF

      else if (b==5){ seg_set_digit(1); tm_apply_with_kick(TM_S1_PCT); }        // 러닝 1단
      else if (b==6){ seg_set_digit(2); tm_apply_with_kick(TM_S2_PCT); }        // 러닝 2단
      else if (b==7){ seg_set_digit(3); tm_apply_with_kick(TM_S3_PCT); }        // 러닝 3단
      else if (b==8){ tm_stop_coast();  seg_set_digit(0); }                     // 러닝 OFF

      else if (b==9) { on_press_S9();  }                                        // 모드키1
      else if (b==10){ on_press_S10(); }                                        // 모드키2
      else if (b==11){ on_press_S11(); }                                        // +30s
      else if (b==12){ on_press_S12(); }                                        // -30s
    }

    if (uart_rx_flag) {                                     // UART 이벤트 처리
      uart_rx_flag = 0;                                     // 플래그 클리어
      HAL_UART_Transmit(&huart1, (uint8_t*)&received_char, 1, 100); // 에코

      if (received_char=='T' || received_char==84){         // 'T'→ 러닝 긴급정지
        tm_stop_coast();  seg_set_digit(0);                 // 러닝 정지+표시 0
        lcd_set_cursor(0,0); lcd_print("Emergency Stop");   // LCD 표시
        lcd_set_cursor(0,1); lcd_print("Treadmill       ");
      }
      else if (received_char=='F' || received_char==70){  // 'F'→ 팬 긴급정지
        sr3_set_led_count(0); fan_stop_coast();             // 팬 정지+LED off
        lcd_set_cursor(0,0); lcd_print("Emergency Stop");  // LCD 표시
        lcd_set_cursor(0,1); lcd_print("Fan             ");
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) // 전압스케일1
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* ===== TIM1 초기화 ===== */
static void MX_TIM1_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();                             // TIM1 클럭 인가

  __HAL_RCC_GPIOA_CLK_ENABLE();                            // GPIOA 클럭 인가
  GPIO_InitTypeDef gi = {0};                               // GPIO 설정 구조체
  gi.Mode  = GPIO_MODE_AF_PP;                              // 대체기능 푸시풀
  gi.Pull  = GPIO_NOPULL;                                  // 풀업/다운 없음
  gi.Speed = GPIO_SPEED_FREQ_LOW;                          // 저속

  gi.Pin = PWM_CH1_PIN; gi.Alternate = PWM_CH1_AF; HAL_GPIO_Init(PWM_CH1_PORT, &gi); // PA8 AF1=TIM1_CH1
  gi.Pin = PWM_CH4_PIN; gi.Alternate = PWM_CH4_AF; HAL_GPIO_Init(PWM_CH4_PORT, &gi); // PA11 AF1=TIM1_CH4

  htim1.Instance = TIM1;                                   // TIM1 선택
  htim1.Init.Prescaler         = PWM_TIM_PRESCALER;        // PSC
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;       // 업카운터
  htim1.Init.Period            = PWM_TIM_PERIOD;           // ARR
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;   // 분주 없음
  htim1.Init.RepetitionCounter = 0;                        // 반복카운터 0
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // ARP 비활성
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler(); // PWM 초기화

  TIM_OC_InitTypeDef s = {0};                              // 출력비교 설정
  s.OCMode       = TIM_OCMODE_PWM1;                        // PWM1 모드
  s.Pulse        = 0;                                      // 초기 듀티 0
  s.OCPolarity   = TIM_OCPOLARITY_HIGH;                    // 정상극성
  s.OCNPolarity  = TIM_OCNPOLARITY_HIGH;                   // N극성
  s.OCFastMode   = TIM_OCFAST_DISABLE;                     // Fast off
  s.OCIdleState  = TIM_OCIDLESTATE_RESET;                  // Idle=0
  s.OCNIdleState = TIM_OCNIDLESTATE_RESET;                 // N Idle=0

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &s, TIM_CHANNEL_1) != HAL_OK) Error_Handler(); // CH1 구성
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &s, TIM_CHANNEL_4) != HAL_OK) Error_Handler(); // CH4 구성
}

/* ===== USART1 초기화(HC-05 @9600) ===== */
static void MX_USART1_UART_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();                           // USART1 클럭 인가

  huart1.Instance          = USART1;                       // USART1
  huart1.Init.BaudRate     = 9600;                         // 9600bps
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;           // 8비트
  huart1.Init.StopBits     = UART_STOPBITS_1;              // 1스톱
  huart1.Init.Parity       = UART_PARITY_NONE;             // 패리티 없음
  huart1.Init.Mode         = UART_MODE_TX_RX;              // 송수신
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;          // 하드플로우 없음
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;         // 오버샘플 16
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;// 1비트 샘플링 X
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // 고급기능 X
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();   // 초기화

  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);                 // 인터럽트 우선순위
  HAL_NVIC_EnableIRQ(USART1_IRQn);                         // 인터럽트 Enable
}

/* ===== GPIO 초기화 ===== */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();                            // GPIOA 클럭
  __HAL_RCC_GPIOB_CLK_ENABLE();                            // GPIOB 클럭

  GPIO_InitTypeDef g={0};                                  // 공용 GPIO 구조체

  /* SR1 (버튼 행 구동, 구 SR0) */
  g.Mode  = GPIO_MODE_OUTPUT_PP; g.Pull  = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW; // PP, NoPull
  g.Pin   = ROW_SER_PIN; HAL_GPIO_Init(ROW_SER_GPIO, &g);                               // PB0(SER)
  g.Pin   = ROW_CLK_PIN; HAL_GPIO_Init(ROW_CLK_GPIO, &g);                               // PA1(CLK)
  HAL_GPIO_WritePin(ROW_SER_GPIO, ROW_SER_PIN, GPIO_PIN_RESET);                         // 초기 Low
  HAL_GPIO_WritePin(ROW_CLK_GPIO, ROW_CLK_PIN, GPIO_PIN_RESET);                         // 초기 Low

  /* Columns (입력 Pull-up) */
  g.Mode  = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_LOW;       // 입력+풀업
  g.Pin   = COL1_PIN; HAL_GPIO_Init(COL_GPIO, &g);                                      // PA2
  g.Pin   = COL2_PIN; HAL_GPIO_Init(COL_GPIO, &g);                                      // PA7
  g.Pin   = COL3_PIN; HAL_GPIO_Init(COL_GPIO, &g);                                      // PA6
  g.Pin   = COL4_PIN; HAL_GPIO_Init(COL_GPIO, &g);                                      // PA5

  /* SR3 (LED/부저/모터 IN, 구 SR2) */
  g.Mode  = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;   // PP
  g.Pin   = SR3_SER_PIN; HAL_GPIO_Init(SR3_SER_GPIO, &g);                               // PA3
  g.Pin   = SR3_CLK_PIN; HAL_GPIO_Init(SR3_CLK_GPIO, &g);                               // PA4
  HAL_GPIO_WritePin(SR3_SER_GPIO, SR3_SER_PIN, GPIO_PIN_RESET);                         // 초기 Low
  HAL_GPIO_WritePin(SR3_CLK_GPIO, SR3_CLK_PIN, GPIO_PIN_RESET);                         // 초기 Low

  /* SR2 (7-seg, 구 SR1) */
  g.Mode  = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;   // PP
  g.Pin   = SR2_SER_PIN; HAL_GPIO_Init(SR2_SER_GPIO, &g);                               // PB1
  g.Pin   = SR2_CLK_PIN; HAL_GPIO_Init(SR2_CLK_GPIO, &g);                               // PB4
  HAL_GPIO_WritePin(SR2_SER_GPIO, SR2_SER_PIN, GPIO_PIN_RESET);                         // 초기 Low
  HAL_GPIO_WritePin(SR2_CLK_GPIO, SR2_CLK_PIN, GPIO_PIN_RESET);                         // 초기 Low

  /* SR2 CLR (상시 High 유지, 구 SR1_CLR) */
  g.Mode  = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;   // PP
  g.Pin   = SR2_CLR_PIN; HAL_GPIO_Init(SR2_CLR_GPIO, &g);                               // PB5
  HAL_GPIO_WritePin(SR2_CLR_GPIO, SR2_CLR_PIN, GPIO_PIN_SET);                           // High

  /* LCD Soft I2C: PB6(SCL), PB7(SDA) -> Open-Drain + Pull-up, Idle High */
  g.Mode  = GPIO_MODE_OUTPUT_OD; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_LOW;   // OD+Pull-up
  g.Pin   = LCD_SCL_PIN; HAL_GPIO_Init(LCD_SCL_GPIO, &g);                               // PB6
  g.Pin   = LCD_SDA_PIN; HAL_GPIO_Init(LCD_SDA_GPIO, &g);                               // PB7
  HAL_GPIO_WritePin(LCD_SCL_GPIO, LCD_SCL_PIN, GPIO_PIN_SET);                           // Idle High
  HAL_GPIO_WritePin(LCD_SDA_GPIO, LCD_SDA_PIN, GPIO_PIN_SET);                           // Idle High

  /* USART1 (PA9=TX, PA10=RX) AF7 */
  g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW; g.Alternate = GPIO_AF7_USART1; // AF7
  g.Pin = GPIO_PIN_9;  HAL_GPIO_Init(GPIOA, &g);                                              // TX
  g.Pin = GPIO_PIN_10; HAL_GPIO_Init(GPIOA, &g);                                              // RX
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) { }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file; (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
