#include "TWAI_ISO.h"
#include <string.h> // For memcpy, memset

#define PCI_TYPE_SINGLE_FRAME       0x00
#define PCI_TYPE_FIRST_FRAME        0x01
#define PCI_TYPE_CONSECUTIVE_FRAME  0x02
#define PCI_TYPE_FLOW_CONTROL       0x03

#define FC_STATUS_CONTINUE_TO_SEND  0x00
#define FC_STATUS_WAIT              0x01
#define FC_STATUS_OVERFLOW          0x02

#define DEFAULT_N_TIMEOUT_MS 1000

static bool _isoTp_platform_send_can_frame(uint32_t id, const uint8_t* data, uint8_t dlc, uint8_t padding_byte) {
    if (dlc > 8) {
        return false;
    }
    twai_message_t message;
    message.identifier = id;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.data_length_code = 8;

    memcpy(message.data, data, dlc);
    for (uint8_t i = dlc; i < 8; ++i) {
        message.data[i] = padding_byte;
    }
    // Serial.printf("ISO-TP CAN TX: ID=0x%03lX, Data: %02X %02X ...\n", id, message.data[0], message.data[1]);
    if (twai_transmit(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
        return true;
    }
    Serial.printf("ISO-TP ERR: CAN Frame TX failed for ID 0x%03lX\n", id);
    return false;
}

static void _isoTp_reset_link_internal(IsoTpLink_t* link, bool full_init_flag) {
    uint8_t* preserved_recv_buf_ptr = link->receive_buffer_ptr;
    uint16_t preserved_recv_buf_max_len = link->receive_buffer_max_len;
    uint8_t  preserved_padding_byte = link->padding_byte;
    uint8_t  preserved_fc_bs_to_send = link->fc_block_size_to_send;
    uint8_t  preserved_fc_stmin_to_send = link->fc_st_min_to_send_ms;

    uint32_t preserved_n_as = link->n_as_timeout_ms;
    uint32_t preserved_n_ar = link->n_ar_timeout_ms;
    uint32_t preserved_n_bs = link->n_bs_timeout_ms;
    uint32_t preserved_n_br = link->n_br_timeout_ms;
    uint32_t preserved_n_cs = link->n_cs_timeout_ms;
    uint32_t preserved_n_cr = link->n_cr_timeout_ms;

    memset(link, 0, sizeof(IsoTpLink_t));

    link->state = ISOTP_IDLE;
    link->is_session_active = false;
    link->receive_buffer_ptr = preserved_recv_buf_ptr;
    link->receive_buffer_max_len = preserved_recv_buf_max_len;
    link->padding_byte = preserved_padding_byte;
    link->fc_block_size_to_send = preserved_fc_bs_to_send;
    link->fc_st_min_to_send_ms = preserved_fc_stmin_to_send;

    link->n_as_timeout_ms = preserved_n_as;
    link->n_ar_timeout_ms = preserved_n_ar;
    link->n_bs_timeout_ms = preserved_n_bs;
    link->n_br_timeout_ms = preserved_n_br;
    link->n_cs_timeout_ms = preserved_n_cs;
    link->n_cr_timeout_ms = preserved_n_cr;
}

void isoTp_initLink(IsoTpLink_t* link,
                    uint8_t* receive_buffer, uint16_t receive_buffer_size,
                    uint32_t default_n_as_ms_param, uint32_t default_n_ar_ms_param,
                    uint32_t default_n_bs_ms_param, uint32_t default_n_br_ms_param,
                    uint32_t default_n_cs_ms_param, uint32_t default_n_cr_ms_param,
                    uint8_t preferred_fc_bs, uint8_t preferred_fc_stmin_ms) {
    if (!link || !receive_buffer || receive_buffer_size == 0) {
        Serial.println("ISO-TP ERR: initLink - Invalid parameters.");
        return;
    }
    uint8_t* temp_rx_buf = receive_buffer;
    uint16_t temp_rx_size = receive_buffer_size;
    
    link->padding_byte = ISO_TP_DEFAULT_PADDING_BYTE;
    link->fc_block_size_to_send = preferred_fc_bs;
    link->fc_st_min_to_send_ms = preferred_fc_stmin_ms;

    _isoTp_reset_link_internal(link, true);

    link->receive_buffer_ptr = temp_rx_buf;
    link->receive_buffer_max_len = temp_rx_size;

    link->n_as_timeout_ms = (default_n_as_ms_param > 0) ? default_n_as_ms_param : DEFAULT_N_TIMEOUT_MS;
    link->n_ar_timeout_ms = (default_n_ar_ms_param > 0) ? default_n_ar_ms_param : DEFAULT_N_TIMEOUT_MS;
    link->n_bs_timeout_ms = (default_n_bs_ms_param > 0) ? default_n_bs_ms_param : DEFAULT_N_TIMEOUT_MS;
    link->n_br_timeout_ms = (default_n_br_ms_param > 0) ? default_n_br_ms_param : DEFAULT_N_TIMEOUT_MS;
    link->n_cs_timeout_ms = default_n_cs_ms_param;
    link->n_cr_timeout_ms = (default_n_cr_ms_param > 0) ? default_n_cr_ms_param : DEFAULT_N_TIMEOUT_MS;
}

bool isoTp_send(IsoTpLink_t* link,
                uint32_t tx_id, uint32_t expected_fc_or_resp_from_id,
                const uint8_t* payload, uint16_t length) {
    if (!link || !payload || length == 0 || length > ISO_TP_MAX_MESSAGE_SIZE) {
        Serial.printf("ISO-TP ERR: isoTp_send - Invalid params\n");
        return false;
    }
    if (link->is_session_active) {
        Serial.println("ISO-TP WARN: isoTp_send - Session busy");
        return false;
    }

    _isoTp_reset_link_internal(link, false);

    link->is_session_active = true;
    link->local_tx_can_id = tx_id;
    link->remote_rx_can_id = expected_fc_or_resp_from_id;
    link->send_buffer_ptr = payload;
    link->send_total_length = length;
    link->send_offset = 0;
    link->send_sequence_number = 0;
    link->last_action_timestamp_ms = millis();

    uint8_t can_data_buffer[8];

    if (length <= 7) { // Single Frame
        can_data_buffer[0] = (PCI_TYPE_SINGLE_FRAME << 4) | (length & 0x0F);
        memcpy(&can_data_buffer[1], payload, length);
        if (!_isoTp_platform_send_can_frame(link->local_tx_can_id, can_data_buffer, length + 1, link->padding_byte)) {
            _isoTp_reset_link_internal(link, false); return false;
        }
        link->state = ISOTP_IDLE;
        Serial.printf("ISO-TP INFO: SF sent (len %u) to ID 0x%03lX\n", length, link->local_tx_can_id);
        return true;
    } else { // First Frame
        can_data_buffer[0] = (PCI_TYPE_FIRST_FRAME << 4) | ((length >> 8) & 0x0F);
        can_data_buffer[1] = length & 0xFF;
        uint8_t ff_data_len = (8 - 2);
        memcpy(&can_data_buffer[2], payload, ff_data_len);
        if (!_isoTp_platform_send_can_frame(link->local_tx_can_id, can_data_buffer, 8, link->padding_byte)) {
            _isoTp_reset_link_internal(link, false); return false;
        }
        link->send_offset = ff_data_len;
        link->send_sequence_number = 1;
        link->state = ISOTP_SEND_WAIT_FC;
        Serial.printf("ISO-TP INFO: FF sent (total_len %u) to ID 0x%03lX, waiting for FC from 0x%03lX\n", length, link->local_tx_can_id, link->remote_rx_can_id);
        return true;
    }
}

void isoTp_poll(IsoTpLink_t* link) {
    if (!link || !link->is_session_active) {
        return;
    }
    unsigned long current_time_ms = millis();

    if (link->state == ISOTP_SEND_SENDING_CF) {
        if (link->send_offset < link->send_total_length && link->flow_control_cf_to_send_in_block > 0) {
            if (current_time_ms - link->last_action_timestamp_ms >= link->fc_st_min_ms_received) { // Use received STmin
                uint8_t can_data_buffer[8];
                can_data_buffer[0] = (PCI_TYPE_CONSECUTIVE_FRAME << 4) | (link->send_sequence_number & 0x0F);
                uint16_t remaining_payload = link->send_total_length - link->send_offset;
                uint8_t cf_payload_len = (remaining_payload > 7) ? 7 : (uint8_t)remaining_payload;
                memcpy(&can_data_buffer[1], link->send_buffer_ptr + link->send_offset, cf_payload_len);

                if (_isoTp_platform_send_can_frame(link->local_tx_can_id, can_data_buffer, cf_payload_len + 1, link->padding_byte)) {
                    link->last_action_timestamp_ms = current_time_ms;
                    link->send_offset += cf_payload_len;
                    link->send_sequence_number = (link->send_sequence_number + 1) % 16;
                    if (link->fc_block_size_received != 0) {
                        link->flow_control_cf_to_send_in_block--;
                    }
                    if (link->send_offset >= link->send_total_length) {
                        Serial.printf("ISO-TP INFO: All CFs sent for msg to ID 0x%03lX.\n", link->local_tx_can_id);
                        link->state = ISOTP_IDLE;
                    } else if (link->fc_block_size_received != 0 && link->flow_control_cf_to_send_in_block == 0) {
                        Serial.println("ISO-TP INFO: Block sent, waiting for next FC.");
                        link->state = ISOTP_SEND_WAIT_FC;
                    }
                } else {
                    Serial.println("ISO-TP ERR: CF send failed. Resetting link.");
                    _isoTp_reset_link_internal(link, false);
                }
            }
        }
    }

    if (link->state == ISOTP_SEND_WAIT_FC && (current_time_ms - link->last_action_timestamp_ms > link->n_bs_timeout_ms)) {
        Serial.println("ISO-TP WARN: Timeout N_Bs (waiting for FC). Resetting link.");
        _isoTp_reset_link_internal(link, false);
    }
    if (link->state == ISOTP_RECV_WAIT_CF && (current_time_ms - link->last_action_timestamp_ms > link->n_cr_timeout_ms)) {
        Serial.println("ISO-TP WARN: Timeout N_Cr (waiting for CF). Resetting link.");
        _isoTp_reset_link_internal(link, false);
    }
    if (link->state != ISOTP_IDLE && link->is_session_active) {
        uint32_t general_timeout = (link->n_as_timeout_ms > link->n_ar_timeout_ms ? link->n_as_timeout_ms : link->n_ar_timeout_ms) + 500;
        if (link->state == ISOTP_SEND_WAIT_FC) general_timeout = link->n_bs_timeout_ms + 500;
        if (link->state == ISOTP_RECV_WAIT_CF) general_timeout = link->n_cr_timeout_ms + 500;
        if (current_time_ms - link->last_action_timestamp_ms > general_timeout) {
            Serial.println("ISO-TP WARN: General session activity timeout. Resetting link.");
            _isoTp_reset_link_internal(link, false);
        }
    }
}

bool isoTp_receive(IsoTpLink_t* link, const twai_message_t* can_msg,
                   uint16_t* out_message_length, uint8_t** out_message_buffer) {
    if (!link || !can_msg || !out_message_length || !out_message_buffer) return false;
    if (can_msg->data_length_code == 0) return false;

    bool is_for_active_session = link->is_session_active && (can_msg->identifier == link->remote_rx_can_id);
    bool is_potential_new_session_obdii = (link->state == ISOTP_IDLE && !link->is_session_active) &&
                                          (can_msg->identifier >= 0x7E8 && can_msg->identifier <= 0x7EF);

    if (!is_for_active_session && !is_potential_new_session_obdii) {
        return false;
    }

    if (is_potential_new_session_obdii && link->state == ISOTP_IDLE) {
        _isoTp_reset_link_internal(link, false);
        link->is_session_active = true;
        link->remote_rx_can_id = can_msg->identifier;
        if (can_msg->identifier >= 0x7E8 && can_msg->identifier <= 0x7EF) {
            link->local_tx_can_id = can_msg->identifier - 0x08; // Where we send FC
        } else {
            link->local_tx_can_id = 0; // Cannot determine FC target
        }
        Serial.printf("ISO-TP INFO: New incoming session from ID 0x%03lX. FC will target 0x%03lX\n", link->remote_rx_can_id, link->local_tx_can_id);
    }

    link->last_action_timestamp_ms = millis();
    const uint8_t* pci_data = can_msg->data;
    uint8_t pci_type = (pci_data[0] & 0xF0) >> 4;

    switch (pci_type) {
        case PCI_TYPE_SINGLE_FRAME: {
            uint16_t sf_dl = pci_data[0] & 0x0F;
            if (sf_dl > 7 || sf_dl > link->receive_buffer_max_len || can_msg->data_length_code < (sf_dl + 1)) {
                Serial.printf("ISO-TP ERR: SF - invalid length %u or DLC %u\n", sf_dl, can_msg->data_length_code);
                _isoTp_reset_link_internal(link, false); return false;
            }
            if (sf_dl > 0) {
                memcpy(link->receive_buffer_ptr, &pci_data[1], sf_dl);
            }
            link->receive_total_expected_length = sf_dl;
            link->receive_offset = sf_dl;
            *out_message_length = sf_dl;
            *out_message_buffer = link->receive_buffer_ptr;
            Serial.printf("ISO-TP INFO: SF received (len %u) from ID 0x%03lX\n", sf_dl, can_msg->identifier);
            link->state = ISOTP_IDLE;
            return true;
        }
        case PCI_TYPE_FIRST_FRAME: {
            if (link->state != ISOTP_IDLE && link->state != ISOTP_RECV_WAIT_CF) {
                 Serial.printf("ISO-TP WARN: FF received but link not idle (state %d)\n", link->state);
                 return false;
            }
            link->receive_total_expected_length = ((uint16_t)(pci_data[0] & 0x0F) << 8) | pci_data[1];
            if (link->receive_total_expected_length == 0 || link->receive_total_expected_length > ISO_TP_MAX_MESSAGE_SIZE ||
                link->receive_total_expected_length > link->receive_buffer_max_len || link->receive_total_expected_length <= 7) {
                Serial.printf("ISO-TP ERR: FF - invalid total length %u\n", link->receive_total_expected_length);
                _isoTp_reset_link_internal(link, false); return false;
            }
            uint8_t ff_payload_in_frame = can_msg->data_length_code - 2;
            if (ff_payload_in_frame > 6) ff_payload_in_frame = 6;
            if (ff_payload_in_frame > link->receive_total_expected_length) {
                ff_payload_in_frame = link->receive_total_expected_length;
            }
            memcpy(link->receive_buffer_ptr, &pci_data[2], ff_payload_in_frame);
            link->receive_offset = ff_payload_in_frame;
            link->receive_sequence_number_expected = 1;
            link->state = ISOTP_RECV_WAIT_CF;

            if (link->local_tx_can_id == 0) {
                Serial.println("ISO-TP ERR: Cannot send FC, local_tx_can_id (FC target) is not set.");
                _isoTp_reset_link_internal(link, false); return false;
            }
            uint8_t fc_can_data[3];
            fc_can_data[0] = (PCI_TYPE_FLOW_CONTROL << 4) | FC_STATUS_CONTINUE_TO_SEND;
            fc_can_data[1] = link->fc_block_size_to_send;    // Use our configured BS
            fc_can_data[2] = link->fc_st_min_to_send_ms;    // Use our configured STmin
            if (!_isoTp_platform_send_can_frame(link->local_tx_can_id, fc_can_data, 3, link->padding_byte)) {
                Serial.println("ISO-TP ERR: Failed to send FC. Resetting link.");
                _isoTp_reset_link_internal(link, false); return false;
            }
            Serial.printf("ISO-TP INFO: FF received (total_len %u) from ID 0x%03lX. FC (CTS,BS=%u,ST=%ums) sent to ID 0x%03lX. Waiting for CFs.\n",
                    link->receive_total_expected_length, can_msg->identifier, link->fc_block_size_to_send, link->fc_st_min_to_send_ms, link->local_tx_can_id);
            return false;
        }
        case PCI_TYPE_CONSECUTIVE_FRAME: {
            if (link->state != ISOTP_RECV_WAIT_CF) {
                Serial.printf("ISO-TP WARN: CF received but link not waiting for CF (state %d)\n", link->state);
                return false;
            }
            uint8_t sn_received = pci_data[0] & 0x0F;
            if (sn_received != (link->receive_sequence_number_expected % 16)) {
                Serial.printf("ISO-TP ERR: CF - SN mismatch. Expected %u, got %u. Resetting.\n", (link->receive_sequence_number_expected % 16), sn_received);
                _isoTp_reset_link_internal(link, false); return false;
            }
            uint8_t cf_payload_in_frame = can_msg->data_length_code - 1;
            if (link->receive_offset + cf_payload_in_frame > link->receive_total_expected_length) {
                cf_payload_in_frame = link->receive_total_expected_length - link->receive_offset;
            }
            if (link->receive_offset + cf_payload_in_frame > link->receive_buffer_max_len) {
                Serial.printf("ISO-TP ERR: CF - buffer overflow. Resetting.\n");
                _isoTp_reset_link_internal(link, false); return false;
            }
            memcpy(link->receive_buffer_ptr + link->receive_offset, &pci_data[1], cf_payload_in_frame);
            link->receive_offset += cf_payload_in_frame;
            link->receive_sequence_number_expected++;
            if (link->receive_offset >= link->receive_total_expected_length) {
                *out_message_length = link->receive_total_expected_length;
                *out_message_buffer = link->receive_buffer_ptr;
                Serial.printf("ISO-TP INFO: All CFs received. Message complete (len %u) from ID 0x%03lX.\n", *out_message_length, can_msg->identifier);
                link->state = ISOTP_IDLE;
                return true;
            }
            return false;
        }
        case PCI_TYPE_FLOW_CONTROL: {
            if (link->state != ISOTP_SEND_WAIT_FC) {
                Serial.printf("ISO-TP WARN: FC received but link not waiting for FC (state %d)\n", link->state);
                return false;
            }
            uint8_t fc_status = pci_data[0] & 0x0F;
            if (fc_status == FC_STATUS_CONTINUE_TO_SEND) {
                link->fc_block_size_received = pci_data[1]; // Store what they want (BS)
                uint8_t st_min_raw = pci_data[2];           // Store what they want (STmin)
                if (st_min_raw <= 0x7F) {
                    link->fc_st_min_ms_received = st_min_raw;
                } else if (st_min_raw >= 0xF1 && st_min_raw <= 0xF9) {
                    link->fc_st_min_ms_received = 1; // Simplified µs handling
                } else {
                    link->fc_st_min_ms_received = 127; // Invalid STmin
                    Serial.printf("ISO-TP WARN: FC - Invalid STmin value 0x%02X from remote, using 127ms\n", st_min_raw);
                }
                link->flow_control_cf_to_send_in_block = (link->fc_block_size_received == 0) ? 0xFF : link->fc_block_size_received;
                link->state = ISOTP_SEND_SENDING_CF;
                link->last_action_timestamp_ms = millis(); // Reset timer for STmin logic
                Serial.printf("ISO-TP INFO: FC (CTS) received from ID 0x%03lX. Remote BS=%u, Remote STmin=%ums. Sending CFs.\n",
                        can_msg->identifier, link->fc_block_size_received, link->fc_st_min_ms_received);
            } else if (fc_status == FC_STATUS_WAIT) {
                 Serial.printf("ISO-TP INFO: FC (WAIT) received from ID 0x%03lX. Holding CFs.\n", can_msg->identifier);
            } else if (fc_status == FC_STATUS_OVERFLOW) {
                Serial.printf("ISO-TP WARN: FC (OVERFLOW) received from ID 0x%03lX. Aborting send.\n", can_msg->identifier);
                _isoTp_reset_link_internal(link, false);
            } else {
                Serial.printf("ISO-TP WARN: FC - Unknown FlowStatus 0x%02X. Aborting.\n", fc_status);
                 _isoTp_reset_link_internal(link, false);
            }
            return false;
        }
        default:
            Serial.printf("ISO-TP WARN: Unknown PCI type 0x%X from ID 0x%03lX\n", pci_type, can_msg->identifier);
            return false;
    }
    return false;
}

void isoTp_resetLink(IsoTpLink_t* link) {
    if (link) {
        _isoTp_reset_link_internal(link, false);
    }
}

bool isoTp_isBusy(const IsoTpLink_t* link) {
    if (link) {
        return link->is_session_active;
    }
    return false;
}

void isoTp_setPaddingByte(IsoTpLink_t* link, uint8_t padding_val) {
    if (link) {
        link->padding_byte = padding_val;
    }
}