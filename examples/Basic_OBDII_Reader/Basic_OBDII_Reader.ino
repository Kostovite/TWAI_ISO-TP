#include <Arduino.h>
#include "driver/twai.h"
#include "TWAI_ISO.h" 
#include <string.h>

// READER'S ACTUAL CAN PINS
#define CAN_RX_PIN_READER GPIO_NUM_21 
#define CAN_TX_PIN_READER GPIO_NUM_37 

#define OBD_RX_BUFFER_SIZE_READER 256 
#define OBD_RESPONSE_ID_FROM_ECU 0x7E8 
const unsigned long REQUEST_TIMEOUT_MS_FOR_READER = 5000; 

IsoTpLink_t g_obd_link_ctx_reader;                     
uint8_t g_obd_rx_buffer_mem_reader[OBD_RX_BUFFER_SIZE_READER]; 
TaskHandle_t g_can_isotp_task_h_reader = NULL;
unsigned long g_request_sent_timestamp_reader = 0;      
bool g_request_is_active_reader = false;            

typedef struct { uint8_t service_code; uint8_t pid_code; const char* desc_str; } PidToScanInfo;
const PidToScanInfo pids_to_auto_scan[] = {
    {0x09, 0x02, "VIN"}, {0x09, 0x0A, "ECU Name"}, {0x01, 0x0C, "Engine RPM"}, {0x01, 0x0D, "Vehicle Speed"},
    {0x01, 0x04, "Engine Load"}, {0x01, 0x05, "Coolant Temp"}, {0x01, 0x2F, "Fuel Level"}, {0x01, 0x0B, "MAP Sensor"},
    {0x01, 0x0F, "Intake Air Temp"}, {0x01, 0x11, "Throttle Position"}, {0x01, 0x49, "Accelerator Pedal Pos"},
    {0x01, 0x4C, "Commanded Throttle"}, {0x01, 0x06, "Short Term Fuel Trim B1"}, {0x01, 0x07, "Long Term Fuel Trim B1"},
    {0x01, 0x14, "O2 Sensor B1S1"}, {0x01, 0x1F, "Runtime Since Start"}, {0x01, 0x0E, "Timing Advance"}, {0x01, 0x46, "Ambient Air Temp"},
};
const int TOTAL_PIDS_FOR_SCAN = sizeof(pids_to_auto_scan) / sizeof(pids_to_auto_scan[0]);
int current_scan_pid_idx = 0;
unsigned long last_scan_pid_request_time = 0;
const unsigned long SCAN_PID_INTERVAL_MS = 1000; 

void setupReaderCAN();
void readerCanIsotpTask(void* pvParameters);
void sendOBDRequest(uint8_t service_val, uint8_t pid_val, const char* desc_val);

float calc_rpm_val(const uint8_t* d) { return ((d[0] * 256.0) + d[1]) / 4.0; }
float calc_percent_val(const uint8_t* d) { return (d[0] * 100.0) / 255.0; }
float calc_obd_temp_val(const uint8_t* d) { return d[0] - 40.0; } 
float calc_map_kpa_val(const uint8_t* d) { return d[0]; } 
float calc_speed_kmh_val(const uint8_t* d) { return d[0]; } 
float calc_fuel_trim_val(const uint8_t* d) { return ((int8_t)d[0] - 128.0) * 100.0 / 128.0; }
float calc_timing_adv_val(const uint8_t* d) { return (d[0] / 2.0) - 64.0; } 
float calc_runtime_s_val(const uint8_t* d) { return (d[0] * 256.0) + d[1]; }
float calc_o2_volt_val(const uint8_t* d) { return d[0] / 200.0; } 

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("ESP32 OBD-II ISO-TP Auto-Scanner (Reader Pins Corrected)");

    setupReaderCAN();
    isoTp_initLink(&g_obd_link_ctx_reader, g_obd_rx_buffer_mem_reader, sizeof(g_obd_rx_buffer_mem_reader),
                   1000, 1000, 1000, 1000, 0, 150, 0, 0);   
    isoTp_setPaddingByte(&g_obd_link_ctx_reader, 0xAA); 
    Serial.println("ISO-TP Link Init.");

    xTaskCreatePinnedToCore(readerCanIsotpTask, "ReaderCANTask", 4096, NULL, 5, &g_can_isotp_task_h_reader, APP_CPU_NUM);
    Serial.println("ReaderCANTask Created.");
    Serial.println("--------------------");
    Serial.println("Auto-scanning PIDs...");
    Serial.println("--------------------");
}

void loop() {
    if (!g_request_is_active_reader && (millis() - last_scan_pid_request_time > SCAN_PID_INTERVAL_MS)) {
        if (current_scan_pid_idx < TOTAL_PIDS_FOR_SCAN) {
            sendOBDRequest(pids_to_auto_scan[current_scan_pid_idx].service_code, 
                          pids_to_auto_scan[current_scan_pid_idx].pid_code, 
                          pids_to_auto_scan[current_scan_pid_idx].desc_str);
            last_scan_pid_request_time = millis();
            current_scan_pid_idx++;
        } else {
            Serial.println("\n---Scan Cycle Done. Restarting.---\n");
            current_scan_pid_idx = 0; 
            last_scan_pid_request_time = millis(); 
        }
    }

    if (g_request_is_active_reader) {
        if (millis() - g_request_sent_timestamp_reader > REQUEST_TIMEOUT_MS_FOR_READER) {
            const char* desc_timeout_str = (current_scan_pid_idx > 0 && current_scan_pid_idx <= TOTAL_PIDS_FOR_SCAN) ? pids_to_auto_scan[current_scan_pid_idx-1].desc_str : "Unknown";
            Serial.printf("WARN: Timeout for %s. Reset ISO-TP.\n", desc_timeout_str);
            isoTp_resetLink(&g_obd_link_ctx_reader);
            g_request_is_active_reader = false;
            last_scan_pid_request_time = millis(); 
        }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); 
}

void setupReaderCAN() {
    twai_general_config_t gen_cfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN_READER, CAN_RX_PIN_READER, TWAI_MODE_NORMAL);
    twai_timing_config_t time_cfg = TWAI_TIMING_CONFIG_500KBITS(); 
    twai_filter_config_t filter_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL(); 
    gen_cfg.rx_queue_len = 20; gen_cfg.tx_queue_len = 10;
    if (twai_driver_install(&gen_cfg, &time_cfg, &filter_cfg) != ESP_OK) { Serial.println("TWAI Install Fail. Halt."); while(1)delay(10); }
    if (twai_start() != ESP_OK) { Serial.println("TWAI Start Fail. Halt."); while(1)delay(10); }
    Serial.println("TWAI Reader Drv Started.");
}

void sendOBDRequest(uint8_t service_val, uint8_t pid_val, const char* desc_val) {
    if (isoTp_isBusy(&g_obd_link_ctx_reader)) {
        Serial.printf("CMD: NoReq %s - ISO Busy.\n", desc_val);
        return;
    }
    Serial.printf("CMD: Req %s (Svc0x%02X,PID0x%02X)...\n", desc_val, service_val, pid_val);
    uint8_t obd_payload[] = {service_val, pid_val}; 

    if (!isoTp_send(&g_obd_link_ctx_reader, 0x7DF, OBD_RESPONSE_ID_FROM_ECU, obd_payload, sizeof(obd_payload))) { 
        Serial.printf("CMD: FailInitReq %s.\n", desc_val);
    } else {
        g_request_sent_timestamp_reader = millis();  
        g_request_is_active_reader = true;        
    }
}

void readerCanIsotpTask(void* pvParams) {
    Serial.println("TASK: ReaderCANTask Started.");
    twai_message_t can_frame_rx; 
    uint16_t isotp_msg_final_len; 
    uint8_t* isotp_msg_final_buf;  

    for (;;) {
        isoTp_poll(&g_obd_link_ctx_reader);
        if (twai_receive(&can_frame_rx, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (isoTp_receive(&g_obd_link_ctx_reader, &can_frame_rx, &isotp_msg_final_len, &isotp_msg_final_buf)) {
                bool buf_ok = (isotp_msg_final_buf >= g_obd_rx_buffer_mem_reader) && (isotp_msg_final_buf + isotp_msg_final_len <= g_obd_rx_buffer_mem_reader + OBD_RX_BUFFER_SIZE_READER);
                if (!buf_ok) { Serial.println("ERR:ISO-TP Buf Ptr OOR! Reset."); isoTp_resetLink(&g_obd_link_ctx_reader); continue; }
                g_request_is_active_reader = false;
                
                if (isotp_msg_final_len >= 2) { 
                    uint8_t svc_resp = isotp_msg_final_buf[0]; uint8_t pid_echo = isotp_msg_final_buf[1];
                    const uint8_t* data_payload_bytes = &isotp_msg_final_buf[2]; uint8_t data_payload_len = isotp_msg_final_len - 2;

                    if (svc_resp == 0x41) { 
                        switch (pid_echo) {
                            case 0x00: if(data_payload_len >= 4)Serial.printf("VAL:SuppPIDs(01-20):%02X%02X%02X%02X\n",data_payload_bytes[0],data_payload_bytes[1],data_payload_bytes[2],data_payload_bytes[3]); break;
                            case 0x04: if(data_payload_len >= 1)Serial.printf("VAL:EngLoad:%.2f%%\n",calc_percent_val(data_payload_bytes)); break;
                            case 0x05: if(data_payload_len >= 1)Serial.printf("VAL:CoolantTemp:%.0fC\n",calc_obd_temp_val(data_payload_bytes)); break;
                            case 0x06: if(data_payload_len >= 1)Serial.printf("VAL:STFT B1:%.2f%%\n",calc_fuel_trim_val(data_payload_bytes)); break;
                            case 0x07: if(data_payload_len >= 1)Serial.printf("VAL:LTFT B1:%.2f%%\n",calc_fuel_trim_val(data_payload_bytes)); break;
                            case 0x0B: if(data_payload_len >= 1)Serial.printf("VAL:MAP:%.0fkPa\n",calc_map_kpa_val(data_payload_bytes)); break;
                            case 0x0C: if(data_payload_len >= 2)Serial.printf("VAL:EngRPM:%.0fRPM\n",calc_rpm_val(data_payload_bytes)); break;
                            case 0x0D: if(data_payload_len >= 1)Serial.printf("VAL:VehSpd:%.0fkm/h\n",calc_speed_kmh_val(data_payload_bytes)); break;
                            case 0x0E: if(data_payload_len >= 1)Serial.printf("VAL:TimingAdv:%.1fdeg\n",calc_timing_adv_val(data_payload_bytes)); break;
                            case 0x0F: if(data_payload_len >= 1)Serial.printf("VAL:IntakeAirTemp:%.0fC\n",calc_obd_temp_val(data_payload_bytes)); break;
                            case 0x11: if(data_payload_len >= 1)Serial.printf("VAL:ThrottlePos:%.2f%%\n",calc_percent_val(data_payload_bytes)); break;
                            case 0x14: if(data_payload_len >= 2)Serial.printf("VAL:O2S B1S1:%.3fV\n",calc_o2_volt_val(data_payload_bytes)); break;
                            case 0x1F: if(data_payload_len >= 2)Serial.printf("VAL:RunTime:%.0fsec\n",calc_runtime_s_val(data_payload_bytes)); break;
                            case 0x2F: if(data_payload_len >= 1)Serial.printf("VAL:FuelLvl:%.2f%%\n",calc_percent_val(data_payload_bytes)); break;
                            case 0x46: if(data_payload_len >= 1)Serial.printf("VAL:AmbientAirTemp:%.0fC\n",calc_obd_temp_val(data_payload_bytes)); break;
                            case 0x49: if(data_payload_len >= 1)Serial.printf("VAL:AccelPedalPosD:%.2f%%\n",calc_percent_val(data_payload_bytes)); break;
                            case 0x4C: if(data_payload_len >= 1)Serial.printf("VAL:CmdThrottleAct:%.2f%%\n",calc_percent_val(data_payload_bytes)); break;
                        }
                    } else if (svc_resp == 0x49) { 
                        const uint8_t* actual_info_ptr = &data_payload_bytes[1]; 
                        uint8_t actual_info_count = data_payload_len > 0 ? data_payload_len -1 : 0;
                        if (data_payload_len == 0 && pid_echo == 0x00) { actual_info_count = 0; }
                        switch (pid_echo) {
                            case 0x00: 
                                if(data_payload_len>=1&&data_payload_bytes[0]>0&&data_payload_len>=1+data_payload_bytes[0]*4){Serial.printf("VAL:SuppPIDs(09):");for(int k=0;k<data_payload_bytes[0]*4;k++){Serial.printf("%02X",actual_info_ptr[k]);}Serial.println();}
                                else if(data_payload_len>=1&&data_payload_bytes[0]==0){Serial.println("VAL:SuppPIDs(09):(None)");}
                                else if(data_payload_len==0&&isotp_msg_final_len==2){Serial.println("VAL:SuppPIDs(09):(NoInfo)");}
                                break;
                            case 0x02: 
                                Serial.print("VAL:VIN:");
                                if(data_payload_len>0&&data_payload_bytes[0]>0){for(int k=0;k<actual_info_count;k++){if(isprint(actual_info_ptr[k]))Serial.print((char)actual_info_ptr[k]);else Serial.print(".");}}
                                else{Serial.print("(NoVINData)");}
                                Serial.println(); break;
                            case 0x0A: 
                                Serial.print("VAL:ECUName:");
                                if(data_payload_len>0&&data_payload_bytes[0]>0){for(int k=0;k<actual_info_count;k++){if(isprint(actual_info_ptr[k]))Serial.print((char)actual_info_ptr[k]);else Serial.print(".");}}
                                else{Serial.print("(NoECUNameData)");}
                                Serial.println(); break;
                        }
                    } 
                }
                isoTp_resetLink(&g_obd_link_ctx_reader);
            }
        }
    }
}

float f_calc_eng_spd(const uint8_t* d){return((d[0]*256.0)+d[1])/4.0;}
float f_calc_eng_load(const uint8_t* d){return(d[0]*100.0)/255.0;}
float f_calc_manifold_press(const uint8_t* d){return d[0];}
float f_calc_throttle_pos(const uint8_t* d){return(d[0]*100.0)/255.0;}
float f_calc_fuel_lvl(const uint8_t* d){return(d[0]*100.0)/255.0;}
float f_calc_veh_spd(const uint8_t* d){return d[0];}
float f_calc_temp(const uint8_t* d){return d[0]-40.0;}
float f_calc_fuel_trim(const uint8_t* d){return((int8_t)(d[0])-128)*100.0/128.0;}