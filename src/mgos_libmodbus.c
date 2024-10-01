/*
 * Copyright 2021 Suyash Mathema
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#include <stdio.h>
#include <unistd.h>
#include <modbus.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <asm/ioctls.h>

#include "mgos_modbus.h"
#include "mgos_rpc.h"

#if CS_PLATFORM == CS_P_CUSTOM || CS_PLATFORM == CS_P_UNIX || CS_PLATFORM == CS_P_WINDOWS

enum uart_read_states { DISABLED,
                        READ_START,
                        RESP_METADATA,
                        RESP_COMPLETE };

enum MB_VALUE_TYPE {
    MAP_TYPE_HEX,
    MAP_TYPE_FLOAT_32,
    MAP_TYPE_LONG_INV_32
};

struct rpc_info {
    struct mg_rpc_request_info* ri;
    char* map;
    char* map_file;
};

static modbus_t *ctx;

uint8_t mb_error(int errnum)
{
    switch (errnum) {
    case RESP_SUCCESS:
        return RESP_SUCCESS;
    case EMBXILFUN:
        return EX_ILLEGAL_FUNCTION;
    case EMBXILADD:
        return EX_ILLEGAL_DATA_ADDRESS;
    case EMBXILVAL:
        return EX_ILLEGAL_DATA_VALUE;
    case EMBXSFAIL:
        return EX_SLAVE_DEVICE_FAILURE;
    case EMBXACK:
        return RESP_TIMED_OUT; //"Acknowledge";
    case EMBXSBUSY:
        return RESP_TIMED_OUT; //"Slave device or server is busy";
    case EMBXNACK:
        return EX_ILLEGAL_FUNCTION; //"Negative acknowledge";
    case EMBXMEMPAR:
        return EX_SLAVE_DEVICE_FAILURE; //"Memory parity error";
    case EMBXGPATH:
        return EX_SLAVE_DEVICE_FAILURE; //"Gateway path unavailable";
    case EMBXGTAR:
        return EX_SLAVE_DEVICE_FAILURE; //"Target device failed to respond";
    case EMBBADCRC:
        return RESP_INVALID_CRC;
    case EMBBADDATA:
        return EX_ILLEGAL_DATA_VALUE; //"Invalid data";
    case EMBBADEXC:
        return RESP_INVALID_FUNCTION; //"Invalid exception code";
    case EMBMDATA:
        return RESP_INVALID_FUNCTION; //"Too many data";
    case EMBBADSLAVE:
        return RESP_INVALID_SLAVE_ID;
    default:
        return RESP_INVALID_FUNCTION;
    }
}

/*
Read coils from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_coils(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Coils, Address: %.2x", read_address));
    struct mb_request_info info;
    struct mbuf response;
    response.size = read_qty * sizeof(uint16_t);
    response.buf = malloc(response.size);
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_read_bits(ctx, (int)read_address, (int)read_qty, (uint8_t *)response.buf)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_read_coils"));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.read_address = read_address;
    info.read_qty = read_qty;
    
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    free(response.buf);
    return false;
}

/*
Read discrete inputs from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_discrete_inputs(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Discrete Inputs, Address: %.2x", read_address));
    struct mb_request_info info;
    struct mbuf response;
    response.size = read_qty * sizeof(uint16_t);
    response.buf = malloc(response.size);
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_read_input_bits(ctx, (int)read_address, (int)read_qty, (uint8_t *)response.buf)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_read_discrete_inputs"));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.read_address = read_address;
    info.read_qty = read_qty;
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    free(response.buf);
    return false;
}

/*
Read holding registers from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_holding_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Holding Registers, Address: %.2x", read_address));
    struct mb_request_info info;
    struct mbuf response;
    response.size = read_qty * sizeof(uint16_t);
    response.buf = malloc(response.size);
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_read_registers(ctx, (int)read_address, (int)read_qty, (uint16_t *)response.buf)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_read_holding_registers"));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.read_address = read_address;
    info.read_qty = read_qty;
    
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    free(response.buf);
    return false;
}

/*
Read input registers from modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_input_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Input Registers, Address: %.2x", read_address));
    struct mb_request_info info;
    struct mbuf response;
    response.size = read_qty * sizeof(uint16_t);
    response.buf = malloc(response.size);
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_read_input_registers(ctx, (int)read_address, (int)read_qty, (uint16_t *)response.buf)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_read_input_registers"));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.read_address = read_address;
    info.read_qty = read_qty;
    
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    free(response.buf);
    return false;
}

/*
Write coil in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_single_coil(uint8_t slave_id, uint16_t write_address, uint16_t write_value, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Single Coil, Address: %.2x", write_address));
    struct mb_request_info info;
    struct mbuf response;
    response.size = 0;
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_write_register(ctx, (int)write_address, write_value)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_write_single_coil %d", write_address));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.write_address = write_address;
    info.write_qty = 1;
    
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    return false;
}

/*
Write register in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_single_register(uint8_t slave_id, uint16_t write_address, uint16_t write_value, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Single Register, Address: %.2x", write_address));
    return mb_write_single_coil(slave_id, write_address, write_value, cb, cb_arg);
}

/*
Write coils in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_multiple_coils(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                             uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Multiple Coils, Address: %.2x", write_address));
    (void) write_qty;
    struct mb_request_info info;
    struct mbuf response;
    response.size = 0;
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_write_bits(ctx, (int)write_address, len, data)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_write_multiple_coils"));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.write_address = write_address;
    info.write_qty = 1;
    
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    return false;
}

/*
Write registers in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_write_multiple_registers(uint8_t slave_id, uint16_t write_address, uint16_t write_qty,
                                 uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Write Multiple Registers, Address: %.2x", write_address));
    (void)write_qty;
    struct mb_request_info info;
    struct mbuf response;
    response.size = 0;
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_write_bits(ctx, (int)write_address, len, data)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_write_multiple_registers"));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.write_address = write_address;
    info.write_qty = 1;
    
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    return false;
}

/*
Read and write mulitple registers in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_read_write_multiple_registers(uint8_t slave_id, uint16_t read_address, uint16_t read_qty,
                                      uint16_t write_address, uint16_t write_qty,
                                      uint8_t* data, uint8_t len, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Read Write Multiple Registers, Address: %.2x", write_address));
    struct mb_request_info info;
    struct mbuf response;
    response.size = len;
    response.buf = malloc(response.size);
    int status = 0;
    int success = true;
    int ret = modbus_set_slave(ctx, slave_id);
    if(ret < 0){
        success = false;
        status = EINVAL;
    } else {
        if ((status = modbus_write_bits(ctx, (int)write_address, write_qty, data)) < 0) {
            LOG(LL_ERROR, ("MODBUS UART error mb_read_write_multiple_registers"));
            success = false;
            response.len = 0;
        } else {
            response.len = (size_t) status;
            if ((status = modbus_read_input_bits(ctx, (int)read_address, (int)read_qty, (uint8_t *)response.buf)) < 0) {
                LOG(LL_ERROR, ("MODBUS UART error mb_read_write_multiple_registers"));
                success = false;
                response.len = 0;
            } else {
                response.len = (size_t) status;
            }
        }
    }
    status = mb_error(status);
    info.slave_id = slave_id;
    info.read_address = read_address;
    info.read_qty = read_qty;
    
    if (cb != NULL) cb((uint8_t)status, info, response, cb_arg);
    free(response.buf);
    return false;
}

/*
Mask register in modbus slave, callback function is called when response from modbus is completed or timed out.
*/
bool mb_mask_write_register(uint8_t slave_id, uint16_t address, uint16_t and_mask, uint16_t or_mask, mb_response_callback cb, void* cb_arg) {
    LOG(LL_DEBUG, ("Mask Write Register, Address: %.2x", address));
    LOG(LL_INFO, ("Not implemented yet"));
    (void)slave_id;
    (void)address;
    (void)and_mask;
    (void)or_mask;
    (void)cb;
    (void)cb_arg;
    return false;
}

bool mgos_modbus_create(const struct mgos_config_modbus* cfg) {
    (void)cfg;
    char parity_str = 'N';

    if (!mgos_sys_config_get_modbus_enable()) {
        return false;
    }
    
    if (cfg->parity == 1) {
        parity_str = 'E';
    } else if (cfg->parity == 2) {
        parity_str = 'O';
    }
    
    ctx = modbus_new_rtu(cfg->device, cfg->baudrate, parity_str, cfg->data_bits, cfg->stop_bits);
    if (ctx == NULL) {
        LOG(LL_ERROR,("Unable to create the libmodbus context"));
        return false;
    }

    modbus_set_response_timeout(ctx, 0, mgos_sys_config_get_modbus_timeout() * 1000);

    int ret = modbus_set_slave(ctx, cfg->slave_id);
    if(ret < 0){
        LOG(LL_ERROR, ("MODBUS UART%s set slave error",cfg->device));
        return false;
    }

    ret = modbus_connect(ctx);
    if(ret < 0){
        LOG(LL_ERROR, ("MODBUS UART%s connect error",cfg->device));
        return false;
    }

    LOG(LL_DEBUG, ("MODBUS UART%s Baudrate %d, Parity '%c', Stop bits %d",
                   cfg->device, cfg->baudrate,
                   parity_str, cfg->stop_bits));

    return true;
}

/*
  Assuming values from modbus are in sequence BADC
  example - Reading 2 holding registers starting at 156 for unit id 5
  request - 05 03 00 9c 00 02 05 a1
  response - 05 03 04 37(B) a8(A) 42(D) 48(C) 00 f1
*/

long parse_value_long_inverse_32(uint8_t* strt_ptr) {
    union {
        uint8_t c[4];
        long l;
    } u;
    //Setting modbus BADC to C memory BADC for long conversion
    u.c[3] = strt_ptr[0];  //B
    u.c[2] = strt_ptr[1];  //A
    u.c[1] = strt_ptr[2];  //D
    u.c[0] = strt_ptr[3];  //C
    return u.l;
}

float parse_value_float32(uint8_t* strt_ptr) {
    union {
        uint8_t c[4];
        float f;
    } u;
    //Setting modbus BADC to C memory DCBA for float conversion
    u.c[3] = strt_ptr[2];  //D
    u.c[2] = strt_ptr[3];  //C
    u.c[1] = strt_ptr[0];  //B
    u.c[0] = strt_ptr[1];  //A

    return u.f;
}

enum MB_VALUE_TYPE parse_address_info(struct json_token address_info, int* address) {
    *address = -1;
    enum MB_VALUE_TYPE type = MAP_TYPE_HEX;

    json_scanf(address_info.ptr, address_info.len, "%d", address);
    if (*address < 0) {
        char* type_temp = NULL;
        json_scanf(address_info.ptr, address_info.len, "{add: %d, type: %Q}", address, &type_temp);
        if (type_temp != NULL) {
            if (strcmp(type_temp, "float") == 0) {
                type = MAP_TYPE_FLOAT_32;
            } else if (strcmp(type_temp, "long_inv") == 0) {
                type = MAP_TYPE_LONG_INV_32;
            }
            free(type_temp);
        }
    }
    return type;
}

int get_buffer_offset(uint16_t read_start_address, uint8_t byte_count, uint16_t required_address) {
    int read_qty = byte_count / 2;
    int max_read_address = read_start_address + read_qty - 1;
    if (required_address < read_start_address || required_address >= max_read_address) {
        LOG(LL_INFO, ("Invalid address: %d, address out of range, start address - %d, byte count - %d",
                      required_address, read_start_address, byte_count));
        return -1;
    }
    int diff = required_address - read_start_address;
    if (diff % 2 != 0) {
        LOG(LL_INFO, ("Invalid address: %d, address value not consistent, start address - %d, byte count - %d",
                      required_address, read_start_address, byte_count));
        return -1;
    }
    return diff * 2 + 3;
}

//Caller needs to free the returned attribute value string
char* get_attribute_value(struct mbuf* mb_reponse, uint16_t read_start_address, struct json_token attr_info) {
    int required_address = -1;
    enum MB_VALUE_TYPE type = parse_address_info(attr_info, &required_address);
    if (required_address < 0) {
        LOG(LL_INFO, ("Cannot find address in modbus response"));
        return NULL;
    }

    int offset = get_buffer_offset(read_start_address, (uint8_t)mb_reponse->buf[2], required_address);
    LOG(LL_DEBUG, ("Attribute info - offset: %d, address: %d, type: %d", offset, required_address, type));
    if (offset < 0) {
        return NULL;
    }
    uint8_t* start_position = (uint8_t*)mb_reponse->buf + offset;
    char* res = NULL;
    switch (type) {
        case MAP_TYPE_FLOAT_32:
            mg_asprintf(&res, 0, "%.2f", parse_value_float32(start_position));
            break;
        case MAP_TYPE_LONG_INV_32:
            mg_asprintf(&res, 0, "%ld", parse_value_long_inverse_32(start_position));
            break;
        case MAP_TYPE_HEX:
        default:
            mg_asprintf(&res, 0, "\"0x%.2x%.2x%.2x%.2x\"", *start_position,
                        *(start_position + 1), *(start_position + 2), *(start_position + 3));
            break;
    }
    return res;
}

char* set_resp_json(struct mbuf* json_buf, char* offset, const char* key,
                    int key_len, const char* value, int value_len) {
    if (json_buf == NULL || key == NULL || value == NULL) {
        return offset;
    }
    int p = 0;
    if (json_buf->len == 0) {
        if ((p = mbuf_insert(json_buf, 0, "{}", 2)) <= 0) {
            return NULL;
        }
        offset = json_buf->buf + 1;
    } else {
        if ((p = mbuf_insert(json_buf, (int)(offset - json_buf->buf), ",", 1)) <= 0) {
            return NULL;
        }
        offset += p;
    }
    char* kv = NULL;
    mg_asprintf(&kv, 0, "\"%.*s\":%.*s", key_len, key, value_len, value);
    if (kv == NULL) {
        return NULL;
    }
    if ((p = mbuf_insert(json_buf, (int)(offset - json_buf->buf), kv, strlen(kv))) <= 0) {
        return NULL;
    }
    offset += p;
    free(kv);
    return offset;
}

char* mb_map_register_response(const char* json_map, struct mbuf* mb_resp, struct mb_request_info* info) {
    LOG(LL_INFO, ("Map modbus response to json"));
    void* h = NULL;
    char* offset = NULL;
    struct json_token attr_name, attr_info;
    struct mbuf resp_buf;
    mbuf_init(&resp_buf, strlen(json_map) * 2);
    while ((h = json_next_key(json_map, strlen(json_map), h, ".", &attr_name, &attr_info)) != NULL) {
        char* attr_value = NULL;
        if ((attr_value = get_attribute_value(mb_resp, info->read_address, attr_info)) != NULL) {
            LOG(LL_VERBOSE_DEBUG, ("Attribute value for %.*s: %s", attr_name.len, attr_name.ptr, attr_value));
            if ((offset = set_resp_json(&resp_buf, offset, attr_name.ptr, attr_name.len,
                                        attr_value, strlen(attr_value))) == NULL) {
                LOG(LL_ERROR, ("Unable to create modbus mapped response json"));
                mbuf_free(&resp_buf);
                return NULL;
            }
            free(attr_value);
        }
    }
    char* resp = NULL;
    if (resp_buf.len > 0) {
        resp = strndup(resp_buf.buf, resp_buf.len);
    }
    mbuf_free(&resp_buf);
    return resp;
}

char* mb_map_register_responsef(const char* json_file, struct mbuf* mb_resp, struct mb_request_info* info) {
    char* map_str = json_fread(json_file);
    if (map_str == NULL) {
        LOG(LL_ERROR, ("Error reading modbus json map file"));
        return NULL;
    }
    char* resp = mb_map_register_response(map_str, mb_resp, info);
    free(map_str);
    return resp;
}

char* mb_resp_to_str(struct mbuf response) {
    int len = response.len * 2 + 3;
    int resp_count = 0;
    char* resp = malloc(len);
    memset(resp, '\0', len);
    resp_count += sprintf(resp + resp_count, "\"");
    for (size_t i = 0; i < response.len && resp_count < len; i++) {
        resp_count += sprintf(resp + resp_count, "%.2x", response.buf[i]);
    }
    resp_count += sprintf(resp + resp_count, "\"");
    return resp;
}

void rpc_mb_resp_cb(uint8_t status, struct mb_request_info info, struct mbuf response, void* param) {
    struct rpc_info* rpc_i = (struct rpc_info*)param;
    char* resp = NULL;
    LOG(LL_INFO, ("Modbus.Read rpc response, status: %#02x", status));
    if (status == RESP_SUCCESS) {
        if (rpc_i->map != NULL) {
            resp = mb_map_register_response(rpc_i->map, &response, &info);
        } else if (rpc_i->map_file != NULL) {
            resp = mb_map_register_responsef(rpc_i->map_file, &response, &info);
        } else {
            resp = mb_resp_to_str(response);
        }
    } else {
        resp = mb_resp_to_str(response);
    }

    if (resp == NULL) {
        mg_rpc_send_errorf(rpc_i->ri, 400, "Invalid json map");
    } else {
        mg_rpc_send_responsef(rpc_i->ri, "{resp_code:%d, data:%s}", status, resp);
    }

    free(resp);
    free(rpc_i->map);
    free(rpc_i->map_file);
    free(rpc_i);
}

static void rpc_modbus_read_handler(struct mg_rpc_request_info* ri, void* cb_arg,
                                    struct mg_rpc_frame_info* fi, struct mg_str args) {
    LOG(LL_INFO, ("Modbus.Read rpc called, payload: %.*s", (int)args.len, args.p));
    int func = -1, id = -1, start = -1, qty = -1;
    char *map_file = NULL, *map = NULL;
    json_scanf(args.p, args.len, ri->args_fmt, &func, &id, &start, &qty, &map_file, &map);
    if (func <= 0) {
        mg_rpc_send_errorf(ri, 400, "Unsupported function code");
        goto out;
    }
    if (id <= 0) {
        mg_rpc_send_errorf(ri, 400, "Slave id is required");
        goto out;
    }
    if (start < 0) {
        mg_rpc_send_errorf(ri, 400, "Read start address is required");
        goto out;
    }
    if (qty <= 0) {
        mg_rpc_send_errorf(ri, 400, "Read quantity is required");
        goto out;
    }

    struct rpc_info* rpc_i = malloc(sizeof(struct rpc_info));
    rpc_i->ri = ri;
    rpc_i->map = map;
    rpc_i->map_file = map_file;

    bool resp = false;
    if ((uint8_t)func == FUNC_READ_COILS) {
        resp = mb_read_coils((uint8_t)id, (uint16_t)start, (uint16_t)qty, rpc_mb_resp_cb, rpc_i);
    } else if ((uint8_t)func == FUNC_READ_DISCRETE_INPUTS) {
        resp = mb_read_discrete_inputs((uint8_t)id, (uint16_t)start, (uint16_t)qty, rpc_mb_resp_cb, rpc_i);
    } else if ((uint8_t)func == FUNC_READ_HOLDING_REGISTERS) {
        resp = mb_read_holding_registers((uint8_t)id, (uint16_t)start, (uint16_t)qty, rpc_mb_resp_cb, rpc_i);
    } else if ((uint8_t)func == FUNC_READ_INPUT_REGISTERS) {
        resp = mb_read_input_registers((uint8_t)id, (uint16_t)start, (uint16_t)qty, rpc_mb_resp_cb, rpc_i);
    }
    if (!resp) {
        mg_rpc_send_errorf(ri, 400, "Unable to execute modbus request");
        free(map);
        free(map_file);
        free(rpc_i);
    }
out:
    (void)cb_arg;
    (void)fi;
    return;
}

bool mgos_modbus_init(void) {
    LOG(LL_DEBUG, ("Initializing modbus"));
    if (!mgos_sys_config_get_modbus_enable())
        return true;
    if (!mgos_modbus_create(&mgos_sys_config.modbus)) {
        return false;
    }
    mg_rpc_add_handler(mgos_rpc_get_global(), "Modbus.Read",
                       "{func: %d, id:%d, start:%d, qty:%d, filename:%Q, json_map:%Q}",
                       rpc_modbus_read_handler, NULL);
    return true;
}

bool mgos_modbus_connect() {
    if (!mgos_sys_config_get_modbus_enable()) {
        return false;
    }
    return mgos_modbus_init();
}
#endif
