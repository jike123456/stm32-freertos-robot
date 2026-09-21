/**
 ****************************************************************************************************
 * @file        lcd_task.c
 * @author      Kaoya (烤鸭)
 * @version     V1.0
 * @date        2026-02-04
 * @brief       LCD 显示任务：电池电压、温度、堆、任务监控表
 * @details
 * 1) 静态 UI：上电后绘制一次（标题栏/状态栏/Heap/表格框架）
 * 2) 动态 UI：周期刷新（电压/温度/heap/任务状态与栈使用率/心跳）
 *
 * @platform    STM32F407ZGT6 + FreeRTOS + HAL
 * @board       Kaoya Robot 下位机
 *
 * @copyright
 *              Copyright (c) 2025-2035 Kaoya. All rights reserved.
 *
 *
 ****************************************************************************************************
 */
#include "lcd_task.h"

#include "lcd.h"
#include "monitor_task.h"
#include "sensor_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>      
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "protocol.h"

/* =========================
 * 屏幕参数（800x480 横屏）
 * ========================= */
#define LCD_W   800u
#define LCD_H   480u

/* =========================
 * 刷新周期
 * ========================= */
#define LCD_REFRESH_MS      1000u    /* UI 刷新：200ms */

/* =========================
 * UI 布局参数（像素）
 * ========================= */
#define UI_MARGIN           8u

#define UI_TITLE_H          50u
#define UI_STATUS_H         40u
#define UI_HEAP_H           50u
#define UI_TABLE_HEAD_H     36u
#define UI_ROW_H            34u

#define UI_TITLE_Y          0u
#define UI_STATUS_Y         (UI_TITLE_Y + UI_TITLE_H)
#define UI_HEAP_Y           (UI_STATUS_Y + UI_STATUS_H)
#define UI_TABLE_Y          (UI_HEAP_Y + UI_HEAP_H)

#define UI_TABLE_HEAD_Y     (UI_TABLE_Y)
#define UI_TABLE_ROWS_Y     (UI_TABLE_Y + UI_TABLE_HEAD_H)

/* =========================
 * 表格列布局
 * ========================= */
#define COL_TASK_X          (UI_MARGIN + 10)
#define COL_STATE_X         170u
#define COL_STACKUSE_X      330u
#define COL_STACKSIZE_X     610u
#define COL_HB_X            720u

#define STACK_BAR_W         190u
#define STACK_BAR_H         16u

/* =========================
 * 顶部状态栏布局
 * =========================
 */
#define BAT_LBL_X           16u
#define BAT_VAL_X           96u
#define BAT_VAL_W           150u
#define BAT_FULL_MV          3300u   /**< 电池满电电压（mV），用于百分比换算 */

#define TMP_LBL_X           270u
#define TMP_VAL_X           322u
#define TMP_VAL_W           110u

#define CMD_LBL_X           470u
#define CMD_VAL_X           520u
#define CMD_VAL_W           120u

/* =========================
 * 颜色
 * ========================= */
#define C_BG        WHITE
#define C_BORDER    GRAY
#define C_TITLE_BG  LGRAYBLUE
#define C_TITLE_TX  WHITE
#define C_TEXT      BLACK
#define C_SUB_BG    LGRAY
#define C_BAR_BG    LGRAY
#define C_OKBAR     GREEN
#define C_WARNBAR   YELLOW
#define C_ERRBAR    RED

/* =========================
 * 任务行定义
 * ========================= */
/**
 * @brief UI 表格行绑定：任务别名 + 心跳变量
 * @note  name 需与 Monitor 注册的 alias 完全一致
 */
typedef struct {
    const char *name;              /**< 任务别名（用于查表） */
    volatile uint32_t *hb_ptr;     /**< 心跳计数器指针（可为空） */
    uint32_t last_hb;              /**< 上次采样心跳值 */
} UiRow_t;

/* 固定行顺序（与 UI 表一致） */
static UiRow_t s_rows[] = {
    {"COMM",   &g_hb_comm,    0},
    {"CTRL",   &g_hb_control, 0},
    {"LOG",    &g_hb_log,     0},
    {"LCD",    &g_hb_lcd,     0},
    {"LED",    &g_hb_led,     0},
    {"IWDG",   &g_hb_iwdg,    0},
    {"MON",    NULL,          0},   /* 目前没 g_hb_mon，就先不显示 */
    {"UPLINK", &g_hb_uplink,  0},
    {"SENSOR", &g_hb_sensor,  0},
};

#define ROW_COUNT   ((uint32_t)(sizeof(s_rows) / sizeof(s_rows[0])))

/* ============================================================
 * 工具函数
 * ============================================================ */

/**
 * @brief 画一个面板：先填充，再画边框
 * @param x1 左上角 x
 * @param y1 左上角 y
 * @param x2 右下角 x
 * @param y2 右下角 y
 * @param border 边框色
 * @param fill 填充色
 */
static void UI_DrawPanel(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                         uint16_t border, uint16_t fill)
{
    lcd_fill(x1, y1, x2, y2, fill);
    lcd_draw_rectangle(x1, y1, x2, y2, border);
}

/**
 * @brief FreeRTOS 任务状态转字符串
 * @param st 任务状态枚举
 * @return 常量字符串
 */
static const char* UI_StateStr(eTaskState st)
{
    switch (st) {
    case eRunning:   return "Running";
    case eReady:     return "Ready";
    case eBlocked:   return "Blocked";
    case eSuspended: return "Suspended";
    case eDeleted:   return "Deleted";
    default:         return "Unknown";
    }
}

/**
 * @brief 获取任务栈高水位（剩余最小空闲，单位：words）
 * @param h 任务句柄
 * @return 高水位（words）；不支持时返回 0
 */
static UBaseType_t UI_GetHwm(TaskHandle_t h)
{
    if (!h) return 0;

#if defined(INCLUDE_uxTaskGetStackHighWaterMark2) && (INCLUDE_uxTaskGetStackHighWaterMark2 == 1)
    return uxTaskGetStackHighWaterMark2(h);
#elif defined(INCLUDE_uxTaskGetStackHighWaterMark) && (INCLUDE_uxTaskGetStackHighWaterMark == 1)
    return uxTaskGetStackHighWaterMark(h);
#else
    return 0;
#endif
}

/**
 * @brief 画进度条（栈使用率）并在右侧显示百分比
 * @param x 起点 x
 * @param y 起点 y
 * @param w 宽度
 * @param h 高度
 * @param pct 百分比 0~100
 * @param fill_color 填充色
 * @param bg_color 背景色
 * @param border_color 边框色
 * @param text_color 文本色
 */
static void UI_DrawBarWithPercent(uint16_t x, uint16_t y,
                                  uint16_t w, uint16_t h,
                                  uint8_t pct,
                                  uint16_t fill_color,
                                  uint16_t bg_color,
                                  uint16_t border_color,
                                  uint16_t text_color)
{
    if (pct > 100) pct = 100;

    /* 外框 */
    lcd_draw_rectangle(x, y, x + w, y + h, border_color);

    /* 内部背景（留 1px 边） */
    if (w > 2 && h > 2) {
        lcd_fill(x + 1, y + 1, x + w - 1, y + h - 1, bg_color);
    }

    /* 填充条 */
    uint16_t inner_w = (w > 2) ? (w - 2) : 0;
    uint16_t inner_h = (h > 2) ? (h - 2) : 0;
    uint16_t fill_w  = (uint16_t)((uint32_t)inner_w * pct / 100u);

    if (fill_w > 0 && inner_h > 0) {
        lcd_fill(x + 1, y + 1, x + 1 + fill_w, y + 1 + inner_h, fill_color);
    }

    /* 百分比字符串 */
    char buf[8];
    snprintf(buf, sizeof(buf), "%u%%", (unsigned)pct);
    lcd_show_string(x + w + 8, y - 2, 60, h + 6, 16, buf, text_color);
}

/**
 * @brief 在 Monitor 快照里按 alias 查找任务信息
 * @param list 快照数组
 * @param n 数组元素个数
 * @param alias 目标别名
 * @param out 输出结构体
 * @return 1 找到；0 未找到
 */
static int UI_FindByAlias(const MonTaskInfo_t *list, uint32_t n,
                          const char *alias, MonTaskInfo_t *out)
{
    if (!list || !alias || !out) return 0;
    for (uint32_t i = 0; i < n; i++) 
	{
        if (list[i].alias && (strcmp(list[i].alias, alias) == 0)) 
		{
            *out = list[i];
            return 1;
        }
    }
    return 0;
}

/* ============================================================
 * 画静态 UI：只画一次
 * ============================================================ */
static void UI_DrawStatic(void)
{
    lcd_clear(C_BG);

    /* 标题栏 */
    UI_DrawPanel(0, UI_TITLE_Y, LCD_W - 1, UI_TITLE_Y + UI_TITLE_H - 1, C_BORDER, C_TITLE_BG);
    lcd_show_string(16, 12, 520, 32, 32, "Kaoya Robot Monitor", C_TEXT);

    /* 状态栏 */
    UI_DrawPanel(0, UI_STATUS_Y, LCD_W - 1, UI_STATUS_Y + UI_STATUS_H - 1, C_BORDER, C_SUB_BG);

    lcd_show_string(BAT_LBL_X, UI_STATUS_Y + 12, 80, 24, 16, "Battery:", C_TEXT);
    lcd_show_string(BAT_VAL_X, UI_STATUS_Y + 12, BAT_VAL_W, 24, 16, "----V  --%", C_TEXT);

    lcd_show_string(TMP_LBL_X, UI_STATUS_Y + 12, 60,  24, 16, "Temp:", C_TEXT);
    lcd_show_string(TMP_VAL_X, UI_STATUS_Y + 12, TMP_VAL_W, 24, 16, "----C", C_TEXT);
	
		lcd_show_string(CMD_LBL_X, UI_STATUS_Y + 12,50, 24, 16, "CMD:", C_TEXT);
		lcd_show_string(CMD_VAL_X, UI_STATUS_Y + 12,CMD_VAL_W, 24, 16, "--------", C_TEXT);

    /* Heap 行 */
    UI_DrawPanel(0, UI_HEAP_Y, LCD_W - 1, UI_HEAP_Y + UI_HEAP_H - 1, C_BORDER, C_BG);
    lcd_show_string(16, UI_HEAP_Y + 16, 60, 24, 16, "heap:", C_TEXT);
    lcd_show_string(70, UI_HEAP_Y + 16, 720, 24, 16, "Free: ---- B   Min: ---- B", C_TEXT);

    /* 表格区域 */
    UI_DrawPanel(0, UI_TABLE_Y, LCD_W - 1, LCD_H - 1, C_BORDER, C_BG);

    /* 表头 */
    lcd_fill(0, UI_TABLE_HEAD_Y, LCD_W - 1, UI_TABLE_HEAD_Y + UI_TABLE_HEAD_H - 1, C_SUB_BG);
    lcd_draw_hline(0, UI_TABLE_HEAD_Y + UI_TABLE_HEAD_H - 1, LCD_W, C_BORDER);

    lcd_show_string(COL_TASK_X,      UI_TABLE_HEAD_Y + 10, 100, 24, 16, "task",      C_TEXT);
    lcd_show_string(COL_STATE_X,     UI_TABLE_HEAD_Y + 10, 120, 24, 16, "state",     C_TEXT);
    lcd_show_string(COL_STACKUSE_X,  UI_TABLE_HEAD_Y + 10, 160, 24, 16, "stackuse",  C_TEXT);
    lcd_show_string(COL_STACKSIZE_X, UI_TABLE_HEAD_Y + 10, 90,  24, 16, "stack",     C_TEXT);
    lcd_show_string(COL_HB_X,        UI_TABLE_HEAD_Y + 10, 70,  24, 16, "hb",        C_TEXT);

    /* 列分隔线（粗略） */
    lcd_draw_line(COL_STATE_X - 10,     UI_TABLE_Y, COL_STATE_X - 10,     LCD_H - 1, C_BORDER);
    lcd_draw_line(COL_STACKUSE_X - 10,  UI_TABLE_Y, COL_STACKUSE_X - 10,  LCD_H - 1, C_BORDER);
    lcd_draw_line(COL_STACKSIZE_X - 10, UI_TABLE_Y, COL_STACKSIZE_X - 10, LCD_H - 1, C_BORDER);
    lcd_draw_line(COL_HB_X - 10,        UI_TABLE_Y, COL_HB_X - 10,        LCD_H - 1, C_BORDER);

    /* 初始化每一行（task 名不变，动态字段给占位） */
    for (uint32_t i = 0; i < ROW_COUNT; i++) {
        uint16_t y = UI_TABLE_ROWS_Y + (uint16_t)(i * UI_ROW_H);

        lcd_draw_hline(0, y + UI_ROW_H - 1, LCD_W, C_BORDER);

        lcd_show_string(COL_TASK_X, y + 8, 140, 24, 16, (char*)s_rows[i].name, C_TEXT);
        lcd_show_string(COL_STATE_X, y + 8, 140, 24, 16, "--------", C_TEXT);

        UI_DrawBarWithPercent(COL_STACKUSE_X, y + 9, STACK_BAR_W, STACK_BAR_H,
                              0, C_OKBAR, C_BAR_BG, C_BORDER, C_TEXT);

        lcd_show_string(COL_STACKSIZE_X, y + 8, 100, 24, 16, "----", C_TEXT);
        lcd_show_string(COL_HB_X, y + 8, 70, 24, 16, "--", C_TEXT);
    }
}

/* =========================
 * 动态刷新
 * ========================= */

/**
 * @brief 刷新动态区域：电池/温度/heap/任务表
 */
static void UI_UpdateDynamicReal(void)
{
    /* 1) 取传感器快照 */
    proto_status_t st;
    memset(&st, 0, sizeof(st));
    SensorSnap_Get(&st);

    /* 2) 顶部：Battery (mV) */
    {
        char batbuf[24];
		
        uint32_t mv = (uint32_t)st.battery_mv;
		if (mv > BAT_FULL_MV) mv = BAT_FULL_MV;
		
		/* 计算电压显示 */
        uint32_t v_int = mv / 1000u;
        uint32_t v_dec = mv % 1000u;
		
		/* 计算百分比：0~BAT_FULL_MV 映射到 0~100%（四舍五入） */
		uint32_t pct = 0u;
		if (BAT_FULL_MV > 0u)
		{
			pct = (mv * 100u + (BAT_FULL_MV / 2u)) / BAT_FULL_MV;
			if (pct > 100u) pct = 100u;
		}
		
        (void)snprintf(batbuf, sizeof(batbuf), "%lu.%03luV  %lu%%",
                   (unsigned long)v_int, (unsigned long)v_dec,
                   (unsigned long)pct);

        lcd_fill(BAT_VAL_X, UI_STATUS_Y + 10, BAT_VAL_X + 150, UI_STATUS_Y + 34, C_SUB_BG);
				   
        lcd_show_string(BAT_VAL_X, UI_STATUS_Y + 12, 160, 24, 16, batbuf, C_TEXT);
    }

    /* 3) 顶部：Temp（℃*100 -> xx.xxC） */
    {
        char tmpbuf[16];
        int32_t t100 = (int32_t)st.temp;
        int32_t t_int = t100 / 100;
        int32_t t_dec = t100 % 100;
        if (t_dec < 0) t_dec = -t_dec;
        snprintf(tmpbuf, sizeof(tmpbuf), "%ld.%02ldC", (long)t_int, (long)t_dec);

        lcd_fill(TMP_VAL_X, UI_STATUS_Y + 10, TMP_VAL_X + 110, UI_STATUS_Y + 34, C_SUB_BG);
        lcd_show_string(TMP_VAL_X, UI_STATUS_Y + 12, 110, 24, 16, tmpbuf, C_TEXT);
    }
		
		/* 显示速度指令状态 */
		{
				const bool cmd_timeout =
						((st.status_flags & PROTO_STATUS_FLAG_CMD_TIMEOUT) != 0u);

				/* 清除上一次显示，防止TIMEOUT切换到OK后留下字符 */
				lcd_fill(CMD_VAL_X,
								 UI_STATUS_Y + 10,
								 CMD_VAL_X + CMD_VAL_W,
								 UI_STATUS_Y + 34,
								 C_SUB_BG);

				if (cmd_timeout)
				{
						lcd_show_string(CMD_VAL_X,
														UI_STATUS_Y + 12,
														CMD_VAL_W,
														24,
														16,
														"TIMEOUT",
														C_ERRBAR);
				}
				else
				{
						lcd_show_string(CMD_VAL_X,
														UI_STATUS_Y + 12,
														CMD_VAL_W,
														24,
														16,
														"OK",
														C_OKBAR);
				}
		}

    /* 4) heap */
#if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
    {
        size_t free_heap = xPortGetFreeHeapSize();
        size_t min_heap  = xPortGetMinimumEverFreeHeapSize();

        char heapbuf[64];
        snprintf(heapbuf, sizeof(heapbuf), "Free: %lu B   Min: %lu B",
                 (unsigned long)free_heap, (unsigned long)min_heap);

        lcd_fill(70, UI_HEAP_Y + 10, LCD_W - 10, UI_HEAP_Y + UI_HEAP_H - 10, C_BG);
        lcd_show_string(70, UI_HEAP_Y + 16, 720, 24, 16, heapbuf, C_TEXT);
    }
#endif

    /* 5) 取 Monitor 注册快照 */
    MonTaskInfo_t snap[12];
    uint32_t n = Monitor_GetTaskInfoSnapshot(snap, 12);

    /* 6) 刷新任务表：state / stackuse / stack size / hb */
    for (uint32_t i = 0; i < ROW_COUNT; i++) {

        uint16_t y = UI_TABLE_ROWS_Y + (uint16_t)(i * UI_ROW_H);

        MonTaskInfo_t t;
        int found = UI_FindByAlias(snap, n, s_rows[i].name, &t);

        /* 清该行动态区域（从 state 到右侧） */
        lcd_fill(COL_STATE_X, y + 6, LCD_W - 10, y + UI_ROW_H - 6, C_BG);

        if (!found || !t.h || t.stack_words == 0u) {

            lcd_show_string(COL_STATE_X, y + 8, 140, 24, 16, "N/A", C_TEXT);

            UI_DrawBarWithPercent(COL_STACKUSE_X, y + 9, STACK_BAR_W, STACK_BAR_H,
                                  0, C_OKBAR, C_BAR_BG, C_BORDER, C_TEXT);

            lcd_show_string(COL_STACKSIZE_X, y + 8, 100, 24, 16, "----", C_TEXT);
            lcd_show_string(COL_HB_X, y + 8, 70, 24, 16, "N/A", C_TEXT);
            continue;
        }

        /* state */
        const char *ststr = UI_StateStr(eTaskGetState(t.h));
        lcd_show_string(COL_STATE_X, y + 8, 140, 24, 16, (char*)ststr, C_TEXT);

        /* stackuse：worst_used% = 100 - min_free% */
        UBaseType_t hwm = UI_GetHwm(t.h);  /* words */
        if (hwm > t.stack_words) hwm = t.stack_words;

        uint32_t min_free_pct   = (uint32_t)hwm * 100u / (uint32_t)t.stack_words;
        uint32_t worst_used_pct = 100u - min_free_pct;
        if (worst_used_pct > 100u) worst_used_pct = 100u;

        uint16_t bar_color = C_OKBAR;
        if (worst_used_pct >= 90u) bar_color = C_ERRBAR;
        else if (worst_used_pct >= 80u) bar_color = C_WARNBAR;

        UI_DrawBarWithPercent(COL_STACKUSE_X, y + 9, STACK_BAR_W, STACK_BAR_H,
                              (uint8_t)worst_used_pct, bar_color, C_BAR_BG, C_BORDER, C_TEXT);

        /* stack size：显示 bytes（words * sizeof(StackType_t)） */
        {
            char sbuf[16];
            uint32_t bytes = (uint32_t)t.stack_words * (uint32_t)sizeof(StackType_t);
            snprintf(sbuf, sizeof(sbuf), "%luB", (unsigned long)bytes);
            lcd_show_string(COL_STACKSIZE_X, y + 8, 100, 24, 16, sbuf, C_TEXT);
        }

        /* hb：判断是否卡死（计数不变则 STALL） */
        if (s_rows[i].hb_ptr) {
            uint32_t cur = *(s_rows[i].hb_ptr);
            if (cur == s_rows[i].last_hb) {
                lcd_show_string(COL_HB_X, y + 8, 70, 24, 16, "STALL", C_ERRBAR);
            } else {
                lcd_show_string(COL_HB_X, y + 8, 70, 24, 16, "OK", C_TEXT);
            }
            s_rows[i].last_hb = cur;
        } else {
            lcd_show_string(COL_HB_X, y + 8, 70, 24, 16, "--", C_TEXT);
        }
    }
}
/* ============================================================
 * LCD 任务入口
 * ============================================================ */

/**
 * @brief LCD 任务入口函数
 * @param argument FreeRTOS 任务参数（未使用）
 */
void KaoYaApp_LcdTask(void *argument)
{
    (void)argument;

    lcd_init();

    /* 横屏 */
    lcd_display_dir(1);
	
    /* 只画一次静态 UI */
    UI_DrawStatic();

    TickType_t last_wake = xTaskGetTickCount();
	
    for (;;) {
		/* 心跳 */
        MON_HB_LCD();
		
        /* 动态刷新 */ 
        UI_UpdateDynamicReal();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LCD_REFRESH_MS));
    }
}
