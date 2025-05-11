#ifndef TWAI_ISO_H
#define TWAI_ISO_H

#include <Arduino.h>
#include "driver/twai.h"

#define ISO_TP_MAX_MESSAGE_SIZE 4095
#define ISO_TP_DEFAULT_PADDING_BYTE 0xAA

typedef enum {
    ISOTP_IDLE,
    ISOTP_SEND_WAIT_FC,
    ISOTP_SEND_SENDING_CF,
    ISOTP_RECV_WAIT_CF
} IsoTpState_t;

typedef struct {
    IsoTpState_t state;
    uint32_t local_tx_can_id;
    uint32_t remote_rx_can_id;

    const uint8_t* send_buffer_ptr;
    uint16_t send_total_length;
    uint16_t send_offset;
    uint8_t  send_sequence_number;

    uint8_t  fc_block_size_received;
    uint8_t  fc_st_min_ms_received;
    uint8_t  flow_control_cf_to_send_in_block;

    uint8_t  fc_block_size_to_send;
    uint8_t  fc_st_min_to_send_ms;

    uint8_t* receive_buffer_ptr;
    uint16_t receive_buffer_max_len;
    uint16_t receive_total_expected_length;
    uint16_t receive_offset;
    uint8_t  receive_sequence_number_expected;

    unsigned long last_action_timestamp_ms;
    bool          is_session_active;
    uint8_t       padding_byte;

    uint32_t      n_as_timeout_ms;
    uint32_t      n_ar_timeout_ms;
    uint32_t      n_bs_timeout_ms;
    uint32_t      n_br_timeout_ms;
    uint32_t      n_cs_timeout_ms;
    uint32_t      n_cr_timeout_ms;

} IsoTpLink_t;


void isoTp_initLink(IsoTpLink_t* link,
                    uint8_t* receive_buffer, uint16_t receive_buffer_size,
                    uint32_t default_n_as_ms, uint32_t default_n_ar_ms,
                    uint32_t default_n_bs_ms, uint32_t default_n_br_ms,
                    uint32_t default_n_cs_ms, uint32_t default_n_cr_ms,
                    uint8_t preferred_fc_bs, uint8_t preferred_fc_stmin_ms);

bool isoTp_send(IsoTpLink_t* link,
                uint32_t tx_id, uint32_t expected_fc_or_resp_from_id,
                const uint8_t* payload, uint16_t length);

void isoTp_poll(IsoTpLink_t* link);

bool isoTp_receive(IsoTpLink_t* link, const twai_message_t* can_msg,
                   uint16_t* out_message_length, uint8_t** out_message_buffer);

void isoTp_resetLink(IsoTpLink_t* link);

bool isoTp_isBusy(const IsoTpLink_t* link);

void isoTp_setPaddingByte(IsoTpLink_t* link, uint8_t padding_val);

#endif // TWAI_ISO_H