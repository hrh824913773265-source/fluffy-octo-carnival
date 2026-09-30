/*
 * 数据采集系统 —— 主控程序（Keil C51）
 * MCU : STC12C5A60S2 @ 11.0592MHz
 * 功能: 正弦波频率/幅度测量, LCD1602 显示, 按键手动控制,
 *       频率/幅度自动闭环控制, RS232(UART1) + RS485(UART2) 通信, 2 路继电器
 *
 * 引脚分配
 *   P0.0~P0.7 LCD1602 D0~D7 (外接 10k 排阻上拉)
 *   P2.0 RS  P2.1 RW  P2.2 EN            (LCD1602)
 *   P2.3 DIN P2.4 SCLK P2.5 CS           (TLC5615 10 位 DAC -> XR2206 频率控制电压)
 *   P1.5 INC P1.6 U/D  P1.7 CS           (X9C103 数字电位器 -> 幅度控制)
 *   P1.0 ADC0                             (峰值检波输出)
 *   P3.4 T0                               (LM393 过零比较器输出, 计数测频)
 *   P3.0/P3.1 UART1 -> MAX232 -> DB9      (RS232)
 *   P1.2/P1.3 UART2 -> MAX485, P1.4 DE/RE (RS485)
 *   P2.6 继电器1  P2.7 继电器2            (低电平吸合, S8550 驱动)
 *   P3.2 K1 模式  P3.3 K2 选择  P3.5 K3 加  P3.6 K4 减
 */
#include <reg52.h>
#include <intrins.h>

/* ---------- STC12C5A60S2 扩展寄存器 ---------- */
sfr AUXR     = 0x8E;
sfr BRT      = 0x9C;
sfr S2CON    = 0x9A;
sfr S2BUF    = 0x9B;
sfr IE2      = 0xAF;
sfr P1ASF    = 0x9D;
sfr ADC_CONTR= 0xBC;
sfr ADC_RES  = 0xBD;
sfr ADC_RESL = 0xBE;

#define ADC_POWER 0x80
#define ADC_FLAG  0x10
#define ADC_START 0x08
#define S2RI 0x01
#define S2TI 0x02

typedef unsigned char  u8;
typedef unsigned int   u16;
typedef unsigned long  u32;

/* ---------- 引脚 ---------- */
sbit LCD_RS = P2^0;  sbit LCD_RW = P2^1;  sbit LCD_EN = P2^2;
sbit DA_DIN = P2^3;  sbit DA_CLK = P2^4;  sbit DA_CS  = P2^5;
sbit RLY1   = P2^6;  sbit RLY2   = P2^7;
sbit POT_INC= P1^5;  sbit POT_UD = P1^6;  sbit POT_CS = P1^7;
sbit RS485_DE = P1^4;
sbit K_MODE = P3^2;  sbit K_SEL = P3^3;  sbit K_UP = P3^5;  sbit K_DN = P3^6;

/* ---------- 标定常数 (调试时按实测修正) ---------- */
/* XR2206: f = (1/RC)*(1 + R/Rc*(1 - Vc/3)), R=Rc=100k, C=10nF, Vc = code*5/1024
 * => f ≈ 2000 - 1.628*code (Hz)                                       */
#define F_AT_CODE0   2000L
#define F_SLOPE_X1000 1628L        /* Hz/code * 1000 */
#define ADC_VREF_MV  5000L         /* ADC 参考 = VCC */
#define PEAK_OFFSET_MV 0           /* 峰值检波零点修正 */

/* ---------- 全局变量 ---------- */
volatile u16 ms_cnt = 0;
volatile bit  gate_done = 0;
volatile u16 t0_ovf = 0;
u32 freq_hz  = 0;         /* 实测频率 */
u16 amp_mv   = 0;         /* 实测峰值 (mV) */

u16 dac_code = 614;       /* 约 1000Hz */
u8  pot_pos  = 50;        /* X9C103 抽头位置 0~99 */
u16 f_set    = 1000;      /* 目标频率 Hz  (500~2000) */
u16 a_set    = 1500;      /* 目标幅度 mV  (1000~2000) */
bit auto_mode = 0;        /* 0 手动 1 自动 */
bit sel_amp   = 0;        /* 0 调频率 1 调幅度 */

/* 串口接收缓冲 */
u8 rx1_buf[16], rx1_len = 0; bit rx1_ok = 0;
u8 rx2_buf[16], rx2_len = 0; bit rx2_ok = 0;

/* ================= 延时 ================= */
void delay_us(u8 n) { while (n--) { _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); } }
void delay_ms(u16 n) { u16 i; while (n--) for (i = 0; i < 900; i++); }

/* ================= LCD1602 ================= */
void lcd_write(bit rs, u8 dat)
{
    delay_us(50);
    LCD_RS = rs; LCD_RW = 0; P0 = dat;
    LCD_EN = 1; delay_us(2); LCD_EN = 0;
    delay_us(50);
}
#define lcd_cmd(c)  lcd_write(0, c)
#define lcd_dat(d)  lcd_write(1, d)
void lcd_init(void)
{
    delay_ms(20);
    lcd_cmd(0x38); delay_ms(5);
    lcd_cmd(0x38); lcd_cmd(0x0C); lcd_cmd(0x06);
    lcd_cmd(0x01); delay_ms(2);
}
void lcd_str(u8 row, u8 col, char *s)
{
    lcd_cmd((row ? 0xC0 : 0x80) + col);
    while (*s) lcd_dat(*s++);
}

/* ================= TLC5615 10 位 DAC ================= */
void dac_write(u16 code)
{
    u8 i; u16 d;
    if (code > 1023) code = 1023;
    d = code << 2;                 /* 12 位帧: 10 位数据 + 2 位填充 */
    DA_CLK = 0; DA_CS = 0;
    for (i = 0; i < 12; i++) {
        DA_DIN = (d & 0x0800) ? 1 : 0;
        DA_CLK = 1; d <<= 1; DA_CLK = 0;
    }
    DA_CS = 1;
}

/* ================= X9C103 数字电位器 ================= */
void pot_step(bit up, u8 n)
{
    POT_UD = up; POT_CS = 0; delay_us(2);
    while (n--) {
        POT_INC = 1; delay_us(2); POT_INC = 0; delay_us(2);
        if (up) { if (pot_pos < 99) pot_pos++; } else { if (pot_pos > 0) pot_pos--; }
    }
    POT_INC = 1; POT_CS = 1;       /* INC 高时 CS 上升 -> 抽头位置存入 EEPROM */
}
void pot_init(void)
{
    pot_step(0, 100);              /* 先回到 0 端, 再走到 50 */
    pot_pos = 0;
    pot_step(1, 50);
}

/* ================= ADC ================= */
u16 adc_read(u8 ch)
{
    u16 r;
    ADC_CONTR = ADC_POWER | 0x60 | ADC_START | ch;   /* 最快速度 */
    _nop_(); _nop_(); _nop_(); _nop_();
    while (!(ADC_CONTR & ADC_FLAG));
    ADC_CONTR &= ~ADC_FLAG;
    r = ((u16)ADC_RES << 2) | (ADC_RESL & 0x03);
    return r;
}
u16 measure_amp(void)
{
    u8 i; u32 s = 0; long mv;
    for (i = 0; i < 16; i++) s += adc_read(0);
    mv = (long)((s >> 4) * ADC_VREF_MV / 1024) + PEAK_OFFSET_MV;
    return mv < 0 ? 0 : (u16)mv;
}

/* ================= 串口 ================= */
void uart_init(void)
{
    SCON  = 0x50;                  /* UART1 方式1 */
    S2CON = 0x50;                  /* UART2 方式1 */
    BRT   = 0xFD;                  /* 9600bps @11.0592MHz, 12T */
    AUXR  = 0x11;                  /* BRTR=1 启动独立波特率发生器, S1BRS=1 UART1 用 BRT */
    ES = 1; IE2 |= 0x01;
    RS485_DE = 0;                  /* 默认接收 */
}
void u1_putc(u8 c) { SBUF = c; while (!TI); TI = 0; }
void u2_putc(u8 c) { S2BUF = c; while (!(S2CON & S2TI)); S2CON &= ~S2TI; }
void send_str(char *s)
{
    char *p = s;
    while (*p) u1_putc(*p++);
    RS485_DE = 1; delay_us(10);
    while (*s) u2_putc(*s++);
    delay_us(200);                 /* 等最后一字节移出 */
    RS485_DE = 0;
}

void uart1_isr(void) interrupt 4
{
    if (RI) {
        u8 c = SBUF; RI = 0;
        if (c == '\r' || c == '\n') { if (rx1_len) { rx1_buf[rx1_len] = 0; rx1_ok = 1; } }
        else if (!rx1_ok && rx1_len < 15) rx1_buf[rx1_len++] = c;
    }
}
void uart2_isr(void) interrupt 8
{
    if (S2CON & S2RI) {
        u8 c = S2BUF; S2CON &= ~S2RI;
        if (c == '\r' || c == '\n') { if (rx2_len) { rx2_buf[rx2_len] = 0; rx2_ok = 1; } }
        else if (!rx2_ok && rx2_len < 15) rx2_buf[rx2_len++] = c;
    }
}

/* ================= 定时器: T0 计数, T1 1ms 闸门 ================= */
void timer_init(void)
{
    TMOD = 0x15;                   /* T0 方式1 计数器, T1 方式1 定时器 */
    TH0 = TL0 = 0;
    TH1 = 0xFC; TL1 = 0x66;        /* 1ms @11.0592MHz 12T */
    ET0 = 1; ET1 = 1;
    TR0 = 1; TR1 = 1;
}
void t0_isr(void) interrupt 1 { t0_ovf++; }
void t1_isr(void) interrupt 3
{
    TH1 = 0xFC; TL1 = 0x66;
    if (++ms_cnt >= 1000) {        /* 1s 闸门 */
        TR0 = 0;
        freq_hz = ((u32)t0_ovf << 16) | ((u16)TH0 << 8) | TL0;
        TH0 = TL0 = 0; t0_ovf = 0;
        TR0 = 1;
        ms_cnt = 0; gate_done = 1;
    }
}

/* ================= 工具 ================= */
void u16_to_str(u16 v, char *buf, u8 w)   /* 右对齐, 宽度 w */
{
    signed char i;
    for (i = w - 1; i >= 0; i--) { buf[i] = v ? '0' + v % 10 : (i == w - 1 ? '0' : ' '); v /= 10; }
    buf[w] = 0;
}
u16 str_to_u16(u8 *s)
{
    u16 v = 0;
    while (*s >= '0' && *s <= '9') v = v * 10 + (*s++ - '0');
    return v;
}
u16 freq_to_code(u16 f)
{
    long c = (F_AT_CODE0 - (long)f) * 1000L / F_SLOPE_X1000;
    if (c < 0) c = 0; if (c > 1023) c = 1023;
    return (u16)c;
}

/* ================= 显示 ================= */
void show(void)
{
    char l1[17], l2[17], t[6];
    u8 i;
    /* 行1: F:1000Hz A:1.50V */
    u16_to_str((u16)freq_hz, t, 4);
    l1[0]='F'; l1[1]=':'; for (i=0;i<4;i++) l1[2+i]=t[i];
    l1[6]='H'; l1[7]='z'; l1[8]=' '; l1[9]='A'; l1[10]=':';
    l1[11]='0'+amp_mv/1000; l1[12]='.'; l1[13]='0'+amp_mv/100%10; l1[14]='0'+amp_mv/10%10; l1[15]='V'; l1[16]=0;
    /* 行2: MAN F> 1000     / AUT A> 1.50 */
    l2[0]= auto_mode?'A':'M'; l2[1]= auto_mode?'U':'A'; l2[2]= auto_mode?'T':'N'; l2[3]=' ';
    l2[4]= sel_amp?'A':'F'; l2[5]='>'; l2[6]=' ';
    if (!sel_amp) {
        u16 v = auto_mode ? f_set : (u16)(F_AT_CODE0 - (long)dac_code * F_SLOPE_X1000 / 1000);
        u16_to_str(v, t, 4); for (i=0;i<4;i++) l2[7+i]=t[i]; l2[11]='H'; l2[12]='z';
    } else if (auto_mode) {
        l2[7]='0'+a_set/1000; l2[8]='.'; l2[9]='0'+a_set/100%10; l2[10]='0'+a_set/10%10; l2[11]='V'; l2[12]=' ';
    } else {
        u16_to_str(pot_pos, t, 4); for (i=0;i<4;i++) l2[7+i]=t[i]; l2[11]=' '; l2[12]=' ';
    }
    l2[13] = RLY1 ? '-' : '1'; l2[14] = RLY2 ? '-' : '2'; l2[15]=' '; l2[16]=0;
    lcd_str(0, 0, l1);
    lcd_str(1, 0, l2);
}

/* ================= 按键 ================= */
u8 key_scan(void)
{
    static u8 hold = 0;
    u8 k = 0;
    if (!K_MODE) k = 1; else if (!K_SEL) k = 2; else if (!K_UP) k = 3; else if (!K_DN) k = 4;
    if (!k) { hold = 0; return 0; }
    delay_ms(10);
    if (hold == 0) { hold = 1; return k; }            /* 首次按下 */
    if (k >= 3 && ++hold > 30) { hold = 28; return k; } /* 加/减 长按连发 */
    return 0;
}
void key_proc(u8 k)
{
    if (k == 1) { auto_mode = !auto_mode;
                  if (auto_mode) { f_set = (u16)freq_hz; if (f_set < 500) f_set = 500; if (f_set > 2000) f_set = 2000;
                                   a_set = amp_mv; if (a_set < 1000) a_set = 1000; if (a_set > 2000) a_set = 2000; } }
    else if (k == 2) sel_amp = !sel_amp;
    else if (k == 3 || k == 4) {
        bit up = (k == 3);
        if (auto_mode) {
            if (!sel_amp) { if (up && f_set < 2000) f_set += 10; if (!up && f_set > 500) f_set -= 10; }
            else          { if (up && a_set < 2000) a_set += 10; if (!up && a_set > 1000) a_set -= 10; }
        } else {
            if (!sel_amp) { /* 频率升高 -> DAC 码减小 */
                if (up && dac_code >= 6) dac_code -= 6; else if (up) dac_code = 0;
                if (!up) { dac_code += 6; if (dac_code > 1023) dac_code = 1023; }
                dac_write(dac_code);
            } else pot_step(up, 1);
        }
    }
}

/* ================= 自动闭环控制 (每 1s 一次) ================= */
void auto_ctrl(void)
{
    long err, d;
    /* 频率: 比例控制, 增益 0.7, 死区 ±1Hz */
    err = (long)f_set - (long)freq_hz;
    if (err > 1 || err < -1) {
        d = err * 1000L / F_SLOPE_X1000 * 7 / 10;
        if (d == 0) d = err > 0 ? 1 : -1;
        d = (long)dac_code - d;
        if (d < 0) d = 0; if (d > 1023) d = 1023;
        dac_code = (u16)d; dac_write(dac_code);
    }
    /* 幅度: 数字电位器每步约 10mV, 死区 ±10mV */
    err = (long)a_set - (long)amp_mv;
    if (err > 10)  pot_step(1, err > 100 ? 5 : 1);
    if (err < -10) pot_step(0, err < -100 ? 5 : 1);
}

/* ================= 串口命令 =================
 *  F1500   设定目标频率 1500Hz (自动模式)
 *  A1500   设定目标幅度 1.500V (自动模式)
 *  M0/M1   手动/自动
 *  R11/R10 继电器1 吸合/释放, R21/R20 继电器2
 *  ?       查询
 */
void cmd_proc(u8 *s)
{
    u16 v = str_to_u16(s + 1);
    switch (s[0]) {
    case 'F': if (v >= 500 && v <= 2000) { f_set = v; auto_mode = 1; } break;
    case 'A': if (v >= 1000 && v <= 2000) { a_set = v; auto_mode = 1; } break;
    case 'M': auto_mode = (s[1] == '1'); break;
    case 'R': if (s[1] == '1') RLY1 = (s[2] != '1'); if (s[1] == '2') RLY2 = (s[2] != '1'); break;
    }
    send_str("OK\r\n");
}
void report(void)
{
    char b[24], t[6]; u8 i, n = 0;
    b[n++]='F'; b[n++]='=';
    u16_to_str((u16)freq_hz, t, 4); for (i=0;i<4;i++) b[n++]=t[i];
    b[n++]=','; b[n++]='A'; b[n++]='=';
    u16_to_str(amp_mv, t, 4); for (i=0;i<4;i++) b[n++]=t[i];
    b[n++]='m'; b[n++]='V'; b[n++]='\r'; b[n++]='\n'; b[n]=0;
    send_str(b);
}

/* ================= 主程序 ================= */
void main(void)
{
    u8 k;
    RLY1 = 1; RLY2 = 1;            /* 继电器释放 */
    P1ASF = 0x01;                  /* P1.0 作 ADC 输入 */
    ADC_CONTR = ADC_POWER; delay_ms(2);
    lcd_init();
    lcd_str(0, 0, "Data Acq System ");
    lcd_str(1, 0, "  Initializing  ");
    dac_code = freq_to_code(f_set); dac_write(dac_code);
    pot_init();
    uart_init();
    timer_init();
    EA = 1;

    while (1) {
        k = key_scan();
        if (k) { key_proc(k); show(); }
        if (rx1_ok) { cmd_proc(rx1_buf); rx1_len = 0; rx1_ok = 0; }
        if (rx2_ok) { cmd_proc(rx2_buf); rx2_len = 0; rx2_ok = 0; }
        if (gate_done) {
            gate_done = 0;
            amp_mv = measure_amp();
            if (auto_mode) auto_ctrl();
            show();
            report();
        }
    }
}
