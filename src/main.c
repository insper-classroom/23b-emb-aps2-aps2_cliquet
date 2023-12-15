/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"
#include "cycle.h"
#include "lv_area.h"
#include "bike.h"
#include "bike_1.h"
#include "arm_math.h"



/************************************************************************/
/* LCD / LVGL                                                           */
/************************************************************************/

LV_FONT_DECLARE(dseg70);
LV_FONT_DECLARE(hora25);
LV_FONT_DECLARE(temp28);
LV_FONT_DECLARE(velocidade);
LV_FONT_DECLARE(trip);





#define LV_HOR_RES_MAX          (320)
#define LV_VER_RES_MAX          (240)
#define TASK_ADC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)
#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define USART_COM USART1
#define USART_COM_ID ID_USART1

#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1u << BUT_PIO_PIN)



#define TASK_SIMULATOR_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_SIMULATOR_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_LCD_STACK_SIZE      (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY  (tskIDLE_PRIORITY)

#define RAIO 0.508/2
#define VEL_MAX_KMH  5.0f
#define VEL_MIN_KMH  0.5f
//#define RAMP
int ramp = 1;
int acelerando;

/*A static or global variable to store the buffers*/
static lv_disp_draw_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];
static lv_disp_drv_t disp_drv;          /*A variable to hold the drivers. Must be static or global.*/
static lv_indev_drv_t indev_drv;

static lv_obj_t * scr1;  // screen 1
static lv_obj_t * scr2;  // screen 2
static int * tela;
static 	lv_obj_t * playpower;
static	lv_obj_t * butsettings;
static 	lv_obj_t * butup;
static	lv_obj_t * butdown;
static 	lv_obj_t * butmenu;
static 	lv_obj_t * butback;
static lv_obj_t * labelTemp;
static lv_obj_t * labelHora;
static lv_obj_t * labelMin;
static lv_obj_t * labelSeg;
static lv_obj_t * labelvel;
static lv_obj_t * labelTrip;
static lv_obj_t * labelTHora;
static lv_obj_t * labelTMin;
static lv_obj_t * labelTSeg;
static lv_obj_t * labelMHora;
static lv_obj_t * labelMMin;
static lv_obj_t * labelMSeg;
static lv_obj_t * labelSim;
static lv_obj_t * labelvel;
static lv_obj_t * labelAP1;
static lv_obj_t * labelAP2;
static lv_obj_t * labelAP3;
static lv_obj_t * labelAP4;
static lv_obj_t * labelDist;
static lv_obj_t * labelDistfinal;
static lv_obj_t * labelVM;

static uint32_t * inicial_hour;
static uint32_t * inicial_min;
static uint32_t * inicial_sec;

static uint32_t * current_hour;
static uint32_t * current_min;
static uint32_t * current_sec;



static void config_AFEC(Afec *afec, uint32_t afec_id);
volatile char flag_rtc_alarm = 0;


//Semaphore
SemaphoreHandle_t xSemaphoreBut1;
SemaphoreHandle_t xSemaphoreBut2;



// Qeueu 
QueueHandle_t xQueueTELA;
QueueHandle_t xQueueADC;
QueueHandle_t xQueueMSG;
QueueHandle_t xQueueTRIP;




//Struct
typedef struct {
	uint value;
} adcData;

 typedef int32_t lv_value_precise_t;

typedef struct{
	lv_value_precise_t x;
	lv_value_precise_t y;
};


typedef struct{
	int timer_counting;
}timeData;

typedef struct {
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
	uint value;
} flagData;


typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}



/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/



static void event_handler_power(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	adcData time;
	//time.value = 0;
	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
		time.value = 1;
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(xQueueTELA, &time, &xHigherPriorityTaskWoken);
	}

}


static void event_handler_menu(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}


static void event_handler_setting(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);

	if(code == LV_EVENT_CLICKED) {
		LV_LOG_USER("Clicked");
	}
	else if(code == LV_EVENT_VALUE_CHANGED) {
		LV_LOG_USER("Toggled");
	}
}


static void event_handler_back(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	adcData time;
	if(code == LV_EVENT_CLICKED) {
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(xQueueTELA, &time, &xHigherPriorityTaskWoken);
	}
}

static void event_handler_up(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		uint *c = lv_label_get_text(labelTemp);
		uint temp = atoi(c);
		lv_label_set_text_fmt(labelTemp, "%02d", temp + 1);
	}
}


static void event_handler_down(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	if(code == LV_EVENT_CLICKED) {
		uint *c = lv_label_get_text(labelTemp);
		uint temp = atoi(c);
		lv_label_set_text_fmt(labelTemp, "%02d", temp - 1);
	}
}

static void event_handler_trip(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	adcData time;

	if(code == LV_EVENT_CLICKED) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreBut2, &xHigherPriorityTaskWoken);
		// QUEUE ENVIA A FLAG
	}
}

static void event_handler_vel(lv_event_t * e) {
	lv_event_code_t code = lv_event_get_code(e);
	adcData vel;
	int ul_previous_time;
	if(code == LV_EVENT_CLICKED) {
		//ul_previous_time = rtt_read_timer_value(RTT);
		vel.value = 10;
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(xQueueMSG, &vel, &xHigherPriorityTaskWoken);
	
	}
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);

	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreBut1, &xHigherPriorityTaskWoken);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {


	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}




void create_scr(lv_obj_t * screen){

		
	    static lv_style_t style;
		lv_obj_clear_flag(scr1, LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_clear_flag(scr2, LV_OBJ_FLAG_SCROLLABLE);
	    lv_color_t c = lv_palette_main(0x000000);
	    lv_color_t black = lv_color_darken(c, 255);
	    lv_style_init(&style);
	    lv_style_set_bg_color(&style, black);
	    lv_style_set_border_color(&style,black);
	    lv_style_set_border_width(&style, 5);
	    


	   // lv_obj_t * img2 = lv_img_create(scr1);
	   // lv_img_set_src(img2, &bike);
	   // lv_obj_align(img2, LV_ALIGN_TOP_LEFT, 0, 0);
	   
/************************************************************************/
/*				                   Tela 1                               */
/************************************************************************/
		
	    lv_obj_t * img = lv_img_create(scr1);
	    lv_img_set_src(img, &cycle);
	    lv_obj_align(img, LV_ALIGN_TOP_LEFT, 0, 0);
	    // PLAY
	    lv_obj_t * play = lv_btn_create(scr1);
	    lv_obj_add_event_cb(play, event_handler_power, LV_EVENT_ALL, NULL);
	    lv_obj_align(play, LV_ALIGN_BOTTOM_LEFT, 10, 0);
	    lv_obj_add_style(play, &style, 0);
	    
	    

	    playpower = lv_label_create(play);
	    lv_label_set_text(playpower, LV_SYMBOL_PLAY);


		labelMHora = lv_label_create(scr1);
		lv_obj_align(labelMHora, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelMHora, &trip, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelMHora, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelMHora,"%02d:",0);
		lv_obj_align(labelMHora, LV_ALIGN_TOP_RIGHT, -95, 85);
		
		
		labelMMin = lv_label_create(scr1);
		lv_obj_align(labelMMin, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelMMin, &trip, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelMMin, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelMMin,"%02d:",0);
		lv_obj_align(labelMMin, LV_ALIGN_TOP_RIGHT, -48, 85);
		
		labelMSeg = lv_label_create(scr1);
		lv_obj_align(labelMSeg, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelMSeg, &trip, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelMSeg, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelMSeg,"%02d",0);
		lv_obj_align(labelMSeg, LV_ALIGN_TOP_RIGHT, -10, 85);
	    
	    labelVM = lv_label_create(scr1);
	    lv_obj_set_style_text_color(labelVM, lv_color_white(), LV_STATE_DEFAULT);
	    lv_obj_align(labelVM, LV_ALIGN_TOP_RIGHT, -10, 50);
		lv_label_set_text_fmt(labelVM,"Vm: %02dKm/h" , 0);
		
		labelDistfinal = lv_label_create(scr1);
		lv_obj_set_style_text_color(labelDistfinal, lv_color_white(), LV_STATE_DEFAULT);
		lv_obj_align(labelDistfinal, LV_ALIGN_TOP_RIGHT, -10, 110);
		lv_label_set_text_fmt(labelDistfinal,"Dist: %02dm" , 0);

    
		rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	    static lv_point_t line_points[] = { {0 , 0}, {0, LV_VER_RES_MAX} };

		static lv_point_t line_points2[] = { {LV_HOR_RES_MAX, 0}, {LV_HOR_RES_MAX/2, 0} };		
		
		
		
		/*Create style*/
		static lv_style_t style_line;
		lv_style_init(&style_line);
		lv_style_set_line_width(&style_line, 2);
		lv_style_set_line_color(&style_line, lv_color_white());
		lv_style_set_line_rounded(&style_line, true);


/************************************************************************/
/*				                   Tela 2                               */
/************************************************************************/

		/*Create a line and apply the new style*/
		lv_obj_t * line1;
		line1 = lv_line_create(scr2);
		lv_line_set_points(line1, line_points, 2);     /*Set the points*/
		lv_obj_add_style(line1, &style_line, 0);
		lv_obj_center(line1);
			
		lv_obj_t * line2;
		line2 = lv_line_create(scr2);
		lv_line_set_points(line2, line_points2, 2);     
		lv_obj_add_style(line2, &style_line, 0);
		lv_obj_center(line2);			

		


		// imagem da roda
		lv_obj_t * img3 = lv_img_create(scr2);
		lv_img_set_src(img3, &bike_1);
		lv_obj_align(img3, LV_ALIGN_BOTTOM_RIGHT, -10, -50);
	
	
	    // Down
	    lv_obj_t * bown = lv_btn_create(scr2);
	    lv_obj_add_event_cb(bown, event_handler_down, LV_EVENT_ALL, NULL);
	   	lv_obj_align(bown, LV_ALIGN_BOTTOM_RIGHT, -30, 0);
	    lv_obj_add_style(bown, &style, 0);

	    butdown = lv_label_create(bown);
	    lv_label_set_text(butdown,LV_SYMBOL_DOWN );
	    lv_obj_center(butdown);

	
	    // UP
	    lv_obj_t * bup = lv_btn_create(scr2);
	    lv_obj_add_event_cb(bup, event_handler_up, LV_EVENT_ALL, NULL);
	    lv_obj_align_to(bup, bown, LV_ALIGN_LEFT_MID, -80, -12);
	    lv_obj_add_style(bup, &style, 0);
	    butup = lv_label_create(bup);
	    lv_label_set_text(butup, LV_SYMBOL_UP );
	    lv_obj_center(butup);
		

	    
	    
	    

	    labelvel = lv_label_create(scr2);
	    lv_obj_align(labelvel, LV_ALIGN_LEFT_MID, 40, -0);
	    lv_obj_set_style_text_font(labelvel, &velocidade, LV_STATE_DEFAULT);
	    lv_obj_set_style_text_color(labelvel, lv_color_white(), LV_STATE_DEFAULT);
	    lv_label_set_text_fmt(labelvel,"%02d" , 0);
  
  
		// but back
	    lv_obj_t * back = lv_btn_create(scr2);
	    lv_obj_add_event_cb(back, event_handler_back, LV_EVENT_ALL, NULL);
	    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 0, 0);
	    lv_obj_add_style(back, &style, 0);
	    butback = lv_label_create(back);
	    lv_label_set_text(butback,LV_SYMBOL_LEFT );
	    lv_obj_center(butback);
		
		
		// Horario
	    labelHora = lv_label_create(scr2);
	    lv_obj_align(labelHora, LV_ALIGN_LEFT_MID, 35 , -45);
	    lv_obj_set_style_text_font(labelHora, &hora25, LV_STATE_DEFAULT);
	    lv_obj_set_style_text_color(labelHora, lv_color_white(), LV_STATE_DEFAULT);
	    lv_label_set_text_fmt(labelHora,"%02d:",(current_hour));
	    lv_obj_align(labelHora, LV_ALIGN_TOP_RIGHT, -95, 10);
		
		
		labelMin = lv_label_create(scr2);
		lv_obj_align(labelMin, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelMin, &hora25, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelMin, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelMin,"%02d:",(current_min));
		lv_obj_align(labelMin, LV_ALIGN_TOP_RIGHT, -48, 10);
		
		labelSeg = lv_label_create(scr2);
		lv_obj_align(labelSeg, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelSeg, &hora25, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelSeg, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelSeg,"%02d",(current_sec));
		lv_obj_align(labelSeg, LV_ALIGN_TOP_RIGHT, -10, 10);
	    
		// Fun��o TRIP
	    lv_obj_t * Trip = lv_btn_create(scr2);
	    lv_obj_add_event_cb(Trip, event_handler_trip, LV_EVENT_ALL, NULL);
	    lv_obj_align(Trip, LV_ALIGN_TOP_RIGHT, -10, 40);
	    lv_obj_add_style(Trip, &style, 0);
		
		labelTHora = lv_label_create(scr2);
		lv_obj_align(labelTHora, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelTHora, &trip, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelTHora, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelTHora,"%02d:",0);
		lv_obj_align(labelTHora, LV_ALIGN_TOP_RIGHT, -95, 85);
		
		
		labelTMin = lv_label_create(scr2);
		lv_obj_align(labelTMin, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelTMin, &trip, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelTMin, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelTMin,"%02d:",0);
		lv_obj_align(labelTMin, LV_ALIGN_TOP_RIGHT, -48, 85);
		
		labelTSeg = lv_label_create(scr2);
		lv_obj_align(labelTSeg, LV_ALIGN_LEFT_MID, 35 , -45);
		lv_obj_set_style_text_font(labelTSeg, &trip, LV_STATE_DEFAULT);
		lv_obj_set_style_text_color(labelTSeg, lv_color_white(), LV_STATE_DEFAULT);
		lv_label_set_text_fmt(labelTSeg,"%02d",0);
		lv_obj_align(labelTSeg, LV_ALIGN_TOP_RIGHT, -10, 85);
		
		
		

	    labelTrip = lv_label_create(Trip);
	    lv_label_set_text(labelTrip,LV_SYMBOL_GPS);
	    lv_obj_center(labelTrip);
		
		//Velocidade

	   labelTemp = lv_label_create(scr2);
	   lv_obj_set_style_text_font(labelTemp, &velocidade, LV_STATE_DEFAULT);
	   lv_obj_set_style_text_color(labelTemp, lv_color_white(), LV_STATE_DEFAULT);
	   lv_label_set_text_fmt(labelTemp, "%02d", 18);
	   lv_obj_align(labelTemp, LV_ALIGN_BOTTOM_RIGHT, -85, -60);
	   

	   //acelera��o
	   labelAP1 = lv_label_create(scr2);
	   lv_obj_set_style_text_color(labelAP1, lv_color_white(), LV_STATE_DEFAULT);
	   lv_obj_align(labelAP1, LV_ALIGN_BOTTOM_LEFT, 10, -110);
	   
		labelAP2 = lv_label_create(scr2);
		lv_obj_set_style_text_color(labelAP2, lv_color_white(), LV_STATE_DEFAULT);
		lv_obj_align(labelAP2, LV_ALIGN_BOTTOM_LEFT, 120, -110);
		
		labelAP3 = lv_label_create(scr2);
		lv_obj_set_style_text_color(labelAP3, lv_color_white(), LV_STATE_DEFAULT);
		lv_obj_align(labelAP3, LV_ALIGN_BOTTOM_LEFT, 10, -95);
		
		labelAP4 = lv_label_create(scr2);
		lv_obj_set_style_text_color(labelAP4, lv_color_white(), LV_STATE_DEFAULT);
		lv_obj_align(labelAP4, LV_ALIGN_BOTTOM_LEFT, 120, -95);
	   
	    labelDist = lv_label_create(scr2);
	    lv_obj_set_style_text_color(labelDist, lv_color_white(), LV_STATE_DEFAULT);
	    lv_obj_align(labelDist, LV_ALIGN_BOTTOM_LEFT, 20, 00);
	    lv_label_set_text_fmt(labelDist,"%02dm" , 0);
	   
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_update(void *pvParameters) {
	adcData time;
	flagData flag;
	calendar rtc_initial = {2023, 12, 13, 2, 17, 30 , 0};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_SECEN);
	timeData tempo;
	
	flag.value = 0;
	int time_counting = 0;
	uint32_t current_year, current_month, current_day, current_week;
	
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);

	rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	


	for (;;)  {		
		
		if(xSemaphoreTake(xSemaphoreBut1, 1000)){

			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);


			
			lv_label_set_text_fmt(labelHora, "%02d:",current_hour);
			lv_label_set_text_fmt(labelMin,"%02d:", current_min);
			lv_label_set_text_fmt(labelSeg, "%02d", current_sec);
			flag.value+=1;
			tempo.timer_counting++;

			BaseType_t xHigherPriorityTaskWoken = pdTRUE;
			xQueueSendFromISR(xQueueTRIP, &tempo, &xHigherPriorityTaskWoken);		

			vTaskDelay(50);
			//rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 1);
		}
		

		
		if(xQueueReceive(xQueueTELA, &(time), 1000) ){
			if(time.value== 1){
				lv_scr_load(scr2);
			}


			else{
				lv_scr_load(scr1);
			}
	
		}	

	}
}


float kmh_to_hz(float vel, float raio) {
	float f = vel / (2*PI*raio*3.6*0.0254);
	return(f);
}

float diamentro(int aro){
	return aro*0.0254; 
}


static void task_trip(void *pvParameters){
	flagData flag;
	timeData tempo;
	int sec = 0;
	int min = 0;
	int hora = 0;
	int contador = 0;
	int contador2 = 0;
	RTT_init(8000, 0, 0 );
	int msg;
	int dist = 0;

	for(;;){
		
		if(xQueueReceive(xQueueTRIP, &(tempo), 1000) ){
		   // int horas, minutos, segundos;
			if(contador >= 59){
				min ++;
				contador = 0;
			}
			else if(min>= 59){
				hora++;
				min = 0;
			}
			contador++;
			contador2++;
			
			lv_label_set_text_fmt(labelTHora, "%02d:",hora);
			lv_label_set_text_fmt(labelTMin,"%02d:", min);
			lv_label_set_text_fmt(labelTSeg, "%02d", contador);

		}
		if(xSemaphoreTake(xSemaphoreBut2, 1000)){
			uint *dist_info = lv_label_get_text(labelDist);
			uint d = atoi(dist_info);
			lv_label_set_text_fmt(labelMHora, "%02d:",hora);
			lv_label_set_text_fmt(labelMMin,"%02d:", min);
			lv_label_set_text_fmt(labelMSeg, "%02d", contador);
			int dist_final = 3.6*d/(contador2);
			lv_label_set_text_fmt(labelDistfinal, "Dist: %02d m", d);
			lv_label_set_text_fmt(labelVM, "Vm: %02dKm/h", dist_final);
			hora = 0;
			min = 0;
			contador = 0;
			dist = 0;
		}
		if(xQueueReceive(xQueueMSG, &msg, (TickType_t) 0)){
			uint *c = lv_label_get_text(labelTemp);
			uint aro = atoi(c);
			dist += msg*PI*(aro)*0.0254;
			lv_label_set_text_fmt(labelDist, "%02d m", dist);
		}

	}
}



static double vel(int num){
	double x = 1.75*num/4095;
	return x;
	
}


static void task_lcd(void *pvParameters) {
	int px, py;
	scr1  = lv_obj_create(NULL);
	scr2  = lv_obj_create(NULL);
	create_scr(scr1);
	lv_scr_load(scr1);
    pmc_enable_periph_clk(ID_PIOC);
    pio_set_output(PIOC, PIO_PC31, 1, 0, 0);

    float vel = VEL_MAX_KMH;
    float f;
    int ramp_up = 1;
	uint *c = lv_label_get_text(labelTemp);
	uint temp = atoi(c);

	for (;;)  {
		pio_clear(PIOC, PIO_PC31);
		delay_ms(1);
		pio_set(PIOC, PIO_PC31);
		lv_tick_inc(50);
		lv_task_handler();
		uint *c = lv_label_get_text(labelTemp);
		uint aro = atoi(c);
		
		vTaskDelay(50);
		pio_clear(PIOC, PIO_PC31);
		delay_ms(1);
		pio_set(PIOC, PIO_PC31);
		if (ramp){
	
			if (ramp_up) {
				acelerando = 1;
		
			   lv_label_set_text(labelAP1,LV_SYMBOL_UP);
			   lv_label_set_text(labelAP2,LV_SYMBOL_UP);
			   lv_label_set_text(labelAP3,LV_SYMBOL_UP);
			   lv_label_set_text(labelAP4,LV_SYMBOL_UP);
			  
		
		
				vel += 0.5;
				} else {
				lv_label_set_text(labelAP1,LV_SYMBOL_DOWN);
				lv_label_set_text(labelAP2,LV_SYMBOL_DOWN);
				lv_label_set_text(labelAP3,LV_SYMBOL_DOWN);
				lv_label_set_text(labelAP4,LV_SYMBOL_DOWN);
				acelerando = 0;
				vel -= 0.5;
			}

			if (vel >= VEL_MAX_KMH)
			ramp_up = 0;
			else if (vel <= VEL_MIN_KMH)
			ramp_up = 1;
			}else{
			vel = 5;
			acelerando = 2;
		}
		lv_label_set_text_fmt(labelvel, "%02d ", (int)(vel));

				
				

		f = kmh_to_hz(vel, (aro/2));
		int t = (1.0/f); //UTILIZADO 965 como multiplicador ao inv�s de 1000
		//para compensar o atraso gerado pelo Escalonador do freeRTOS
		delay_ms(10);
	
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
		xQueueSendFromISR(xQueueMSG, &t, &xHigherPriorityTaskWoken);
	}
}


/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
	/**LCD pin configure on SPI*/
	pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
	pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
	pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
	pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
	pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
	pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
	
	
	ili9341_init();
	ili9341_backlight_on();
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT,
	};

	/* Configure console UART. */
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
	ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
	ili9341_copy_pixels_to_screen(color_p,  (area->x2 + 1 - area->x1) * (area->y2 + 1 - area->y1));
	
	/* IMPORTANT!!!
	* Inform the graphics library that you are ready with the flushing*/
	lv_disp_flush_ready(disp_drv);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}


void my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	int px, py, pressed;
	
	if (readPoint(&px, &py))
		data->state = LV_INDEV_STATE_PRESSED;
	else
		data->state = LV_INDEV_STATE_RELEASED; 
	
	data->point.x = px;
	data->point.y = py;
}

void configure_lvgl(void) {
	lv_init();
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);
	
	lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
	disp_drv.draw_buf = &disp_buf;          /*Set an initialized buffer*/
	disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
	disp_drv.hor_res = LV_HOR_RES_MAX;      /*Set the horizontal resolution in pixels*/
	disp_drv.ver_res = LV_VER_RES_MAX;      /*Set the vertical resolution in pixels*/

	lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
	
	/* Init input on LVGL */
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_input_read;
	lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
}

void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* board and sys init */
	board_init();
	init();
	configure_console();

	/* LCd, touch and lvgl init*/
	configure_lcd();
	configure_touch();
	configure_lvgl();

	

	
	
	
	 xSemaphoreBut1 = xSemaphoreCreateBinary();
	 if (xSemaphoreBut1 == NULL){
	  printf("falha em criar o semaforo \n");
	  }	    
	xSemaphoreBut2 = xSemaphoreCreateBinary();
	if (xSemaphoreBut2 == NULL){
		printf("falha em criar o semaforo \n");
	}

	xQueueTELA = xQueueCreate(100, sizeof(adcData));
	if (xQueueTELA == NULL)
	printf("falha em criar a queue xQueueADC \n");


	xQueueADC = xQueueCreate(100, sizeof(adcData));
	if (xQueueADC == NULL)
		printf("falha em criar a queue xQueueADC \n");

	xQueueTRIP = xQueueCreate(100, sizeof(adcData));
	if (xQueueTRIP == NULL)
		printf("falha em criar a queue xQueueADC \n");
	xQueueMSG = xQueueCreate(100, sizeof(adcData));
	if (xQueueMSG == NULL)
		printf("falha em criar a queue xQueueADC \n");

	/* Create task to control oled */

	if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	if (xTaskCreate(task_update, "update", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}
	if (xTaskCreate(task_trip, "trip", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create lcd task\r\n");
	}


	

	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){ }
}
