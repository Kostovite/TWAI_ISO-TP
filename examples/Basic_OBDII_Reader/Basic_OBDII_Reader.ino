#include <Arduino.h>
#include "driver/twai.h" // ESP-IDF TWAI driver
#include "TWAI_ISO.h"    // Your ISO-TP layer header

// --- Configuration ---
#define CAN_RX_PIN GPIO_NUM_21 // D21 on your ESP32-WROOM-DA (Verify!)
#define CAN_TX_PIN GPIO_NUM_22 // D22 on your ESP32-WROOM-DA (Verify!)

#define OBD_RX_BUFFER_SIZE 256 // Max size for reassembled ISO-TP messages

// --- Global Variables ---
IsoTpLink_t g_obd_link;                     // Global ISO-TP link context
uint8_t g_obd_rx_buffer[OBD_RX_BUFFER_SIZE]; // Buffer for reassembled messages

TaskHandle_t g_can_isotp_task_handle = NULL;

// Added for Error Recovery
unsigned long g_request_timestamp = 0;      // When the last request was sent
bool g_request_pending = false;            // Whether we're waiting for a response
const unsigned long REQUEST_TIMEOUT_MS = 5000; // 5 second timeout for OBD responses

// --- Function Prototypes ---
void setupCAN();
void can_isotp_task(void* pvParameters);
void requestVIN();
void requestRPM_via_ISO_TP_SF();

// --- Arduino Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Wait for serial
    }
    Serial.println("ESP32 OBD-II ISO-TP Reader Starting (v2)...");

    // Initialize CAN hardware
    setupCAN();

    // Initialize ISO-TP link context
    // Timeout values (N_As, N_Ar, N_Bs, N_Br, N_Cs, N_Cr) - using simplified values.
    // These should be tuned based on ISO 15765-2 and OBD-II requirements.
    // N_Cs is often 0 for sender if not using advanced separation time control.
    // Preferred FC params for FCs *this device sends*: BS=0 (send all), STmin=0ms (fastest)
    isoTp_initLink(&g_obd_link, g_obd_rx_buffer, sizeof(g_obd_rx_buffer),
                   1000, 1000, // N_As_sender, N_Ar_receiver (actually N_Ar for sender waiting for response from receiver)
                   1000, 1000, // N_Bs_sender (waiting for FC), N_Br_receiver (timeout between FC and next CF)
                   0,    150,  // N_Cs_sender (between CFs, usually 0 if STmin used), N_Cr_receiver (waiting for CF)
                   0,    0);   // Our preferred FC_BS = 0, FC_STmin = 0ms
    isoTp_setPaddingByte(&g_obd_link, 0xAA); // Set preferred CAN frame padding

    Serial.println("ISO-TP Link Initialized.");

    // Create FreeRTOS task for CAN RX and ISO-TP polling
    xTaskCreatePinnedToCore(
        can_isotp_task,         // Task function
        "CANISO_TPTask",        // Name of the task
        4096,                   // Stack size in words (increased a bit for safety)
        NULL,                   // Task input parameter
        5,                      // Priority of the task
        &g_can_isotp_task_handle, // Task handle
        APP_CPU_NUM             // Core where the task should run
    );

    Serial.println("CAN/ISO-TP Task Created.");
    Serial.println("------------------------------------");
    Serial.println("Type 'v' to request VIN (multi-frame).");
    Serial.println("Type 'r' to request RPM (single-frame via ISO-TP).");
    Serial.println("------------------------------------");
}

// --- Arduino Loop (Main Task) ---
void loop() {
    // Check for input commands
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'v' || cmd == 'V') {
            requestVIN();
        } else if (cmd == 'r' || cmd == 'R') {
            requestRPM_via_ISO_TP_SF();
        }
    }

    // Error recovery - check for timeout on pending requests
    if (g_request_pending) {
        if (millis() - g_request_timestamp > REQUEST_TIMEOUT_MS) {
            Serial.println("WARN: OBD-II request timeout. Resetting ISO-TP link.");
            isoTp_resetLink(&g_obd_link);
            g_request_pending = false;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Keep main loop responsive
}

// --- CAN Initialization ---
void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Standard OBD-II speed
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // ISO-TP layer will handle ID logic

    g_config.rx_queue_len = 20; // Increased queue for potential bursts during ISO-TP
    g_config.tx_queue_len = 10;

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI Driver installed.");
    } else {
        Serial.println("Failed to install TWAI driver. Halting.");
        while(1); // Halt
    }

    if (twai_start() == ESP_OK) {
        Serial.println("TWAI Driver started.");
    } else {
        Serial.println("Failed to start TWAI driver. Halting.");
        while(1); // Halt
    }
}

// --- ISO-TP Request Functions ---
void requestVIN() {
    if (isoTp_isBusy(&g_obd_link)) {
        Serial.println("CMD: Cannot request VIN - ISO-TP link is busy.");
        return;
    }
    Serial.println("CMD: Requesting VIN (Service 0x09, PID 0x02)...");
    uint8_t vin_payload[] = {0x09, 0x02}; // OBD-II Service 09, PID 02

    if (!isoTp_send(&g_obd_link, 0x7DF, 0x7E8, vin_payload, sizeof(vin_payload))) {
        Serial.println("CMD: Failed to initiate VIN request via ISO-TP.");
    } else {
        Serial.println("CMD: VIN request initiated (FF sent or SF sent).");
        g_request_timestamp = millis();  // Record when request was sent
        g_request_pending = true;        // Mark that we're waiting for a response
    }
}

void requestRPM_via_ISO_TP_SF() {
     if (isoTp_isBusy(&g_obd_link)) {
        Serial.println("CMD: Cannot request RPM - ISO-TP link is busy.");
        return;
    }
    Serial.println("CMD: Requesting RPM (Service 0x01, PID 0x0C) via ISO-TP SF...");
    uint8_t rpm_payload[] = {0x01, 0x0C}; // OBD-II Service 01, PID 0C (Engine RPM)

    if (!isoTp_send(&g_obd_link, 0x7DF, 0x7E8, rpm_payload, sizeof(rpm_payload))) {
        Serial.println("CMD: Failed to initiate RPM SF request via ISO-TP.");
    } else {
        Serial.println("CMD: RPM SF request initiated.");
        g_request_timestamp = millis();  // Record when request was sent
        g_request_pending = true;        // Mark that we're waiting for a response
    }
}


// --- FreeRTOS Task for CAN RX and ISO-TP Polling ---
void can_isotp_task(void* pvParameters) {
    Serial.println("TASK: can_isotp_task started.");
    twai_message_t rx_can_msg;
    uint16_t complete_msg_len;
    uint8_t* complete_msg_buf; // Will point into g_obd_rx_buffer

    for (;;) {
        // 1. Poll ISO-TP link for ongoing transmissions (sending CFs) and timeouts
        isoTp_poll(&g_obd_link);

        // 2. Check for incoming CAN frames
        if (twai_receive(&rx_can_msg, pdMS_TO_TICKS(10)) == ESP_OK) { // Poll with short timeout
            // Optional: Raw CAN frame logging for deep debugging
            // Serial.printf("TASK RAW RX: ID=0x%03lX DLC=%d Data: ", rx_can_msg.identifier, rx_can_msg.data_length_code);
            // for(int i=0; i<rx_can_msg.data_length_code; i++) { Serial.printf("%02X ", rx_can_msg.data[i]); }
            // Serial.println();

            if (isoTp_receive(&g_obd_link, &rx_can_msg, &complete_msg_len, &complete_msg_buf)) {
                // Buffer protection - validate the pointer is within our buffer
                bool buffer_valid = (complete_msg_buf >= g_obd_rx_buffer) &&
                                   (complete_msg_buf + complete_msg_len <= g_obd_rx_buffer + OBD_RX_BUFFER_SIZE);

                if (!buffer_valid) {
                    Serial.println("ERROR: ISO-TP buffer pointer out of range! Resetting link.");
                    isoTp_resetLink(&g_obd_link);
                    // Note: As per specific instructions, g_request_pending is not cleared here.
                    // This means if a corrupt message is received, the timeout in loop() might still trigger.
                    continue; // Skip processing this message
                }

                // A full ISO-TP message has been reassembled!
                Serial.print("TASK: Complete ISO-TP message received! Length: ");
                Serial.println(complete_msg_len);

                // Also clear the request pending flag since we got a response
                g_request_pending = false;

                Serial.print("TASK: OBD Payload: ");
                for (int i = 0; i < complete_msg_len; i++) {
                    Serial.print(complete_msg_buf[i], HEX);
                    Serial.print(" ");
                }
                Serial.println();

                // --- Process the reassembled OBD-II payload ---
                if (complete_msg_len >= 2) { // Must have at least Mode and PID
                    uint8_t service_response = complete_msg_buf[0]; // e.g., 0x41, 0x49
                    uint8_t pid_code_echo = complete_msg_buf[1];

                    Serial.print("TASK: Parsed Service Resp: 0x"); Serial.print(service_response, HEX);
                    Serial.print(", PID Echo: 0x"); Serial.println(pid_code_echo, HEX);

                    if (service_response == 0x49 && pid_code_echo == 0x02 && complete_msg_len > 2) { // VIN Response
                        Serial.print("TASK: VIN: ");
                        for (int i = 2; i < complete_msg_len; i++) {
                            Serial.print((char)complete_msg_buf[i]);
                        }
                        Serial.println();
                    } else if (service_response == 0x41 && pid_code_echo == 0x0C && complete_msg_len >= 4) { // RPM Response
                        // Data bytes for RPM are A and B (at index 2 and 3 of OBD payload)
                        int rpm = ((int)complete_msg_buf[2] * 256 + (int)complete_msg_buf[3]) / 4;
                        Serial.print("TASK: Engine RPM: "); Serial.println(rpm);
                    }
                    // Add more parsers here for other PIDs you request...
                } else if (complete_msg_len > 0) { // Not enough for mode and PID but some data
                     Serial.println("TASK: Received ISO-TP message too short for standard OBD parsing.");
                }

                // Important: Reset the link for the next transaction after processing the message
                // or if an error occurs that makes the current transaction invalid.
                isoTp_resetLink(&g_obd_link);
                g_request_pending = false;  // Clear pending flag when resetting link after successful processing
            }
        }
        // vTaskDelay(pdMS_TO_TICKS(1)); // A very small yield if twai_receive timeout is very short or 0
                                      // Otherwise, the timeout in twai_receive provides yielding.
    }
}