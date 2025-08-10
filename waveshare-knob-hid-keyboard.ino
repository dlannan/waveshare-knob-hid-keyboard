#include "lcd_bsp.h"
#include "cst816.h"
#include "lcd_bl_pwm_bsp.h"
#include "lcd_config.h"
#include "ui.h"
#include "bidi_switch_knob.h"

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

#include <BleKeyboard.h>
#include "esp_sleep.h"

BleKeyboard bleKeyboard;

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x54, 0x43, 0xB2, 0x59, 0x60, 0xF2 };

typedef struct struct_message {
    byte power;
    byte red;
    byte green;
    byte blue;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

static const char * hid_commands[5] = {
  "cmd window",
  "explorer",
  "firefox",
  "build cse",
  "build ot",
};


#define BACKLIGHT_PIN 10  // GPIO10

void turnOnBacklight() {
  setUpdutySubdivide(LCD_PWM_MODE_255);
}

void turnOffBacklight() {
  setUpdutySubdivide(LCD_PWM_MODE_0);
}

static const char *TAG = "encoder";

static lv_obj_t * meter;
lv_meter_indicator_t * needle; 

static lv_obj_t * meter2;
lv_meter_indicator_t * needle2; 

static lv_obj_t * meter3;
static int active_command = 0;
// lv_meter_indicator_t * needle3; 


static lv_obj_t * meter4;
lv_meter_indicator_t * needle4; 

#define EXAMPLE_ENCODER_ECA_PIN    8
#define EXAMPLE_ENCODER_ECB_PIN    7

#define SET_BIT(reg,bit) (reg |= ((uint32_t)0x01<<bit))
#define CLEAR_BIT(reg,bit) (reg &= (~((uint32_t)0x01<<bit)))
#define READ_BIT(reg,bit) (((uint32_t)reg>>bit) & 0x01)
#define BIT_EVEN_ALL (0x00ffffff)

EventGroupHandle_t knob_even_ = NULL;

static knob_handle_t s_knob = 0;

SemaphoreHandle_t mutex; 

RTC_DATA_ATTR int bootCount = 0;
#define THRESHOLD 5000 /* Lower the value, more the sensitivity */

touch_pad_t touchPin;

byte value[4]= {50,25,25,25};   // values of each meter 0= power, 1 =green , 2 red, 3 blue 
int chosen=0;


void lv_example_meter_1(void)
{
    extern lv_obj_t *ui_Screen1;
    meter = lv_meter_create(ui_Screen1);
    lv_obj_add_event_cb(meter, meter_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_center(meter);
    lv_obj_set_size(meter, 200, 200);
    lv_obj_set_pos(meter, -71, 0);
    lv_obj_set_style_bg_opa(meter, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter, 0, LV_PART_MAIN);

    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_range(meter, scale, 0, 100, 270, 135); // Set range from 0 to 10
    lv_meter_set_scale_ticks(meter, scale, 31, 3, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 6, 5, 15, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    /*Add a blue arc to the start*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /*Make the tick lines blue at the start of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /*Add a red arc to the end*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /*Make the tick lines red at the end of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /*Add a needle line indicator*/
    needle = lv_meter_add_needle_line(meter, scale, 4, lv_color_hex(0xFFFFFF), -10); 

   
}

void lv_example_meter_2(void)
{
     extern lv_obj_t *ui_Screen1;
     meter2 = lv_meter_create(ui_Screen1);
     lv_obj_add_event_cb(meter2, meter2_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_center(meter2);
    lv_obj_set_size(meter2, 146, 146);
    lv_obj_set_pos(meter2, 109, 0);
    lv_obj_set_style_bg_opa(meter2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter2, 0, LV_PART_MAIN);

    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter2);
    lv_meter_set_scale_range(meter2, scale, 0, 100, 270, 135); // Set range from 0 to 10
    lv_meter_set_scale_ticks(meter2, scale, 31, 2, 8, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter2, scale, 6, 3, 11, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    /*Add a blue arc to the start*/
    indic = lv_meter_add_arc(meter2, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter2, indic, 0);
    lv_meter_set_indicator_end_value(meter2, indic, 20);

    /*Make the tick lines blue at the start of the scale*/
    indic = lv_meter_add_scale_lines(meter2, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter2, indic, 0);
    lv_meter_set_indicator_end_value(meter2, indic, 20);

    /*Add a red arc to the end*/
    indic = lv_meter_add_arc(meter2, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter2, indic, 80);
    lv_meter_set_indicator_end_value(meter2, indic, 100);

    /*Make the tick lines red at the end of the scale*/
    indic = lv_meter_add_scale_lines(meter2, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter2, indic, 80);
    lv_meter_set_indicator_end_value(meter2, indic, 100);

    /*Add a needle line indicator*/
    needle2 = lv_meter_add_needle_line(meter2, scale, 3, lv_color_hex(0xFFFFFF), -10); 
}

void lv_example_meter_3(void)
{
     extern lv_obj_t *ui_Screen1;
     meter3 = lv_obj_create(ui_Screen1);
     lv_obj_add_event_cb(meter3, meter3_event_cb, LV_EVENT_ALL, NULL);
    // lv_obj_center(meter3);
    // lv_obj_set_size(meter3, 128, 128);
    // lv_obj_set_pos(meter3, 43, -107);
    // lv_obj_set_style_bg_opa(meter3, LV_OPA_TRANSP, LV_PART_MAIN)

    /*Add a scale first*/
    // lv_meter_scale_t * scale = lv_meter_add_scale(meter3);
    // lv_meter_set_scale_range(meter3, scale, 0, 100, 270, 135); // Set range from 0 to 10
    // lv_meter_set_scale_ticks(meter3, scale, 31, 1, 6, lv_palette_main(LV_PALETTE_GREY));
    // lv_meter_set_scale_major_ticks(meter3, scale, 6, 2, 10, lv_color_white(), 10);

    // lv_meter_indicator_t * indic;

    // /*Add a blue arc to the start*/
    // indic = lv_meter_add_arc(meter3, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    // lv_meter_set_indicator_start_value(meter3, indic, 0);
    // lv_meter_set_indicator_end_value(meter3, indic, 20);

    // /*Make the tick lines blue at the start of the scale*/
    // indic = lv_meter_add_scale_lines(meter3, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    // lv_meter_set_indicator_start_value(meter3, indic, 0);
    // lv_meter_set_indicator_end_value(meter3, indic, 20);

    // /*Add a red arc to the end*/
    // indic = lv_meter_add_arc(meter3, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    // lv_meter_set_indicator_start_value(meter3, indic, 80);
    // lv_meter_set_indicator_end_value(meter3, indic, 100);

    // /*Make the tick lines red at the end of the scale*/
    // indic = lv_meter_add_scale_lines(meter3, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    // lv_meter_set_indicator_start_value(meter3, indic, 80);
    // lv_meter_set_indicator_end_value(meter3, indic, 100);

    /*Add a needle line indicator*/
    // needle3 = lv_meter_add_needle_line(meter3, scale, 3, lv_color_hex(0xFFFFFF), -10); 

    lv_obj_set_width(meter3, 128);
    lv_obj_set_height(meter3, 90);
    lv_obj_set_x(meter3, 43);
    lv_obj_set_y(meter3, -127);
    lv_obj_set_align(meter3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(meter3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(meter3, 5, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(meter3, lv_color_hex(0x535151), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(meter3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(meter3, 0, LV_PART_MAIN);

    ui_Label3 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, 43);
    lv_obj_set_y(ui_Label3, -157);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "CMD");
    lv_obj_set_style_text_color(ui_Label3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label3, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_power3 = lv_label_create(ui_Screen1);
    lv_obj_set_width(ui_power3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_power3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_power3, 43);
    lv_obj_set_y(ui_power3, -117);

    lv_obj_set_align(ui_power3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_power3, hid_commands[active_command]);
    lv_obj_set_style_text_color(ui_power3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_power3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_power3, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);    
}

void lv_example_meter_4(void)
{
     extern lv_obj_t *ui_Screen1;
     meter4 = lv_meter_create(ui_Screen1);
     lv_obj_add_event_cb(meter4, meter4_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_center(meter4);
    lv_obj_set_size(meter4, 128, 128);
    lv_obj_set_pos(meter4, 43, 107);
    lv_obj_set_style_bg_opa(meter4, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter4, 0, LV_PART_MAIN);

    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter4);
    lv_meter_set_scale_range(meter4, scale, 0, 100, 270, 135); // Set range from 0 to 10
    lv_meter_set_scale_ticks(meter4, scale, 31, 1, 6, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter4, scale, 6, 2, 10, lv_color_white(), 10);

    lv_meter_indicator_t * indic;

    /*Add a blue arc to the start*/
    indic = lv_meter_add_arc(meter4, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter4, indic, 0);
    lv_meter_set_indicator_end_value(meter4, indic, 20);

    /*Make the tick lines blue at the start of the scale*/
    indic = lv_meter_add_scale_lines(meter4, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
    lv_meter_set_indicator_start_value(meter4, indic, 0);
    lv_meter_set_indicator_end_value(meter4, indic, 20);

    /*Add a red arc to the end*/
    indic = lv_meter_add_arc(meter4, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter4, indic, 80);
    lv_meter_set_indicator_end_value(meter4, indic, 100);

    /*Make the tick lines red at the end of the scale*/
    indic = lv_meter_add_scale_lines(meter4, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
    lv_meter_set_indicator_start_value(meter4, indic, 80);
    lv_meter_set_indicator_end_value(meter4, indic, 100);

    /*Add a needle line indicator*/
    needle4 = lv_meter_add_needle_line(meter4, scale, 3, lv_color_hex(0xFFFFFF), -10); 
}

static void _knob_left_cb(void *arg, void *data)
{
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_,0);
  xEventGroupSetBits(knob_even_,eventBits_);
}
static void _knob_right_cb(void *arg, void *data)
{
  uint8_t eventBits_ = 0;
  SET_BIT(eventBits_,1);
  xEventGroupSetBits(knob_even_,eventBits_);
}

bool shoot=false;
int pos=-96;

volatile uint32_t lastActivityTime = 0;

void IRAM_ATTR onUserInput() {
  lastActivityTime = millis();
}

void setup()
{
  mutex = xSemaphoreCreateMutex();
  Serial.begin(115200);

  delay(1000); // Take some time to open up the Serial Monitor

  Serial.println("Starting BLE work!");  

  bleKeyboard.begin();  
  Touch_Init();
  lcd_lvgl_Init();

  lv_example_meter_1();
  lv_example_meter_2();
  lv_example_meter_3();
  lv_example_meter_4();

  set_active_meter(chosen);
  lcd_bl_pwm_bsp_init(40); // brightness up to 255

  turnOnBacklight();  // Turn it on initially

  knob_even_ = xEventGroupCreate();
  // create knob
  knob_config_t cfg = 
  {
    .gpio_encoder_a = EXAMPLE_ENCODER_ECA_PIN,
    .gpio_encoder_b = EXAMPLE_ENCODER_ECB_PIN,
  };
  s_knob = iot_knob_create(&cfg);

  iot_knob_register_cb(s_knob, KNOB_LEFT, _knob_left_cb, NULL);
  iot_knob_register_cb(s_knob, KNOB_RIGHT, _knob_right_cb, NULL);
  xTaskCreate(user_encoder_loop_task, "user_encoder_loop_task", 3000, NULL, 2, NULL);
  xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
  xTaskCreate(sleep_manager_task, "sleep_manager_task", 3000, NULL, 2, NULL);

  WiFi.mode(WIFI_STA);

  gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

static void sleep_manager_task(void *pvParameters) {
  const uint32_t idleTimeoutMs = 20000;  // 10 seconds idle timeout
  while (true) {
    if ((millis() - lastActivityTime) > idleTimeoutMs) {
      Serial.println("Going into light sleep...");

       esp_sleep_enable_gpio_wakeup();
 
      // Before sleeping
      turnOffBacklight();

      // Enter light sleep
      esp_light_sleep_start();
      
      // After waking
      turnOnBacklight();

      // Light sleep returns here on wake
      Serial.println("Woke from light sleep.");
      lastActivityTime = millis();  // Reset idle timer
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
  }
}

static void user_encoder_loop_task(void *arg)
{
  for(;;)
  {
    EventBits_t even = xEventGroupWaitBits(knob_even_,BIT_EVEN_ALL,pdTRUE,pdFALSE,pdMS_TO_TICKS(5000)); //等待WIFI 连接成功
    if(READ_BIT(even,0))
    { 
      if (xSemaphoreTake(mutex, portMAX_DELAY)) { 
        if(value[chosen]>0) {
          value[chosen]=value[chosen]-2;
          lastActivityTime =  millis();
        }
        switch(chosen) {
          case 0: 
            if(bleKeyboard.isConnected()) 
              bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
            break;

          case 2:
            active_command--;
            if(active_command < 0) active_command = 4;
            break;
        }
    
        myData.power=value[0];
        myData.red = map(value[2],0,100,0,255);
        myData.blue = map(value[3],0,100,0,255);
        myData.green = map(value[1],0,100,0,255);

        // Send message via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        xSemaphoreGive(mutex); 
      }
    }

    if(READ_BIT(even,1))
    {
      if (xSemaphoreTake(mutex, portMAX_DELAY)) { 
        value[chosen]=value[chosen]+2;
        if(value[chosen]>100) {
          value[chosen] = 100;
          lastActivityTime =  millis();
        }

        switch(chosen) {
          case 0: 
            if(bleKeyboard.isConnected()) 
              bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
            break;

          case 2:
            active_command ++;
            if(active_command > 4) active_command = 0;
            break;
        }

        myData.power=value[0];
        myData.red = map(value[2],0,100,0,255);
        myData.blue = map(value[3],0,100,0,255);
        myData.green = map(value[1],0,100,0,255);  


        // Send message via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
        xSemaphoreGive(mutex); 
      }
    }
  }
}

static void example_lvgl_port_task(void *arg)
{
  for(;;)
  {
    lv_timer_handler();

    if (xSemaphoreTake(mutex, portMAX_DELAY)) { 
        
      lv_label_set_text(ui_power,String(value[0]).c_str());
      lv_meter_set_indicator_value(meter, needle, value[0]);
      lv_arc_set_value(ui_Arc1, value[0]);

      lv_label_set_text(ui_power1,String(value[1]).c_str());
      lv_meter_set_indicator_value(meter2, needle2, value[1]);
      lv_arc_set_value(ui_Arc2, value[1]);

      lv_label_set_text(ui_power3,hid_commands[active_command]);
//      lv_meter_set_indicator_value(meter3, needle3, value[2]);
//      lv_arc_set_value(ui_Arc3, value[2]);

      lv_label_set_text(ui_power2,String(value[3]).c_str());
      lv_meter_set_indicator_value(meter4, needle4, value[3]);
      lv_arc_set_value(ui_Arc4, value[3]);

      
      lv_color_t color = lv_color_make(map(value[2],0,100,0,255), map(value[1],0,100,0,255), map(value[3],0,100,0,255));
      lv_obj_set_style_bg_color(ui_colorPNL, color, LV_PART_MAIN);

      xSemaphoreGive(mutex); 
    }
    vTaskDelay(pdMS_TO_TICKS(1));  
  }
  
}

void meter_event_cb(lv_event_t * e)
{
  if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
    chosen = 0;
    set_active_meter(chosen);
  }
}

void meter2_event_cb(lv_event_t * e)
{
  if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
    chosen = 1;
    set_active_meter(chosen);
  }
}

void run_command()
{
  Serial.print("Running Command: ");
  Serial.println( hid_commands[active_command]);
  switch(active_command)
  {
    case 0:
      // if(bleKeyboard.isConnected()) 
      //   bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
      break;
    case 1:
      if(bleKeyboard.isConnected()) 
        bleKeyboard.write(KEY_MEDIA_LOCAL_MACHINE_BROWSER);
      break;
    case 2: 
      if(bleKeyboard.isConnected()) 
        bleKeyboard.write(KEY_MEDIA_WWW_HOME);
      break;    
  }
  delay(200); // Stop multiple quick launches.
}

void meter3_event_cb(lv_event_t * e)
{
  if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
    if(chosen == 2) run_command(); // Already command set, run it!!
    chosen = 2;
    set_active_meter(chosen);
  }
}

void meter4_event_cb(lv_event_t * e)
{
  if(lv_event_get_code(e) == LV_EVENT_CLICKED) {
    chosen = 3;
    set_active_meter(chosen);
  }
}

void set_active_meter(int index)
{
    // Resetiraj svim metrima border (sakrij)
    lv_obj_set_style_border_width(meter,  0, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter2, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter3, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(meter4, 0, LV_PART_MAIN);

    // Postavi aktivnom metru border
    switch(index) {
        case 0:
            lv_obj_set_style_border_width(meter,  2, LV_PART_MAIN);
            lv_obj_set_style_border_color(meter,  lv_color_hex(0x574509), LV_PART_MAIN);
            break;
        case 1:
            lv_obj_set_style_border_width(meter2, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(meter2, lv_color_hex(0x574509), LV_PART_MAIN);
            break;
        case 3:
            lv_obj_set_style_border_width(meter4, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(meter4, lv_color_hex(0X574509), LV_PART_MAIN);
            break;
        case 2:
            lv_obj_set_style_border_width(meter3, 2, LV_PART_MAIN);
            lv_obj_set_style_border_color(meter3, lv_color_hex(0x574509), LV_PART_MAIN);
            break;
    }
    lastActivityTime =  millis();
}

void loop()
{

}
