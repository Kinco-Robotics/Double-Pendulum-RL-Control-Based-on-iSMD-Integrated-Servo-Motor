# MControl API Documentation

## 1. Overview

MControl provides drive communication APIs as a C ABI dynamic library.

The current API uses a single-connection model:

- A dynamic library instance maintains only one communication connection.
- The connection represents only the physical communication channel and is no longer bound to a device station number.
- Each read, write, or business operation explicitly receives `station`.
- Except for string-release and legacy diagnostic APIs, all public business operations return a `char*` JSON string.
- After reading the JSON, the caller must call `free_string` to release the returned string.

This allows a single RS485/CAN bus to access multiple station numbers through the same connection, making it suitable for encapsulating multi-joint devices.

## 2. General Conventions

### 2.1 JSON Response Structure

All JSON APIs use a unified structure:

```json
{
  "ok": true,
  "data": 90,
  "unit": "deg",
  "error": null
}
```

On failure:

```json
{
  "ok": false,
  "data": null,
  "unit": null,
  "error": "read 0x6063_00 timeout"
}
```

On a successful write:

```json
{
  "ok": true,
  "data": null,
  "unit": null,
  "error": null
}
```

Field descriptions:

- `ok`: A Boolean value indicating whether the call succeeded.
- `data`: The returned data. Read APIs usually return a number or string; successful write APIs return `null`.
- `unit`: The unit of the returned data. It is `null` when no business-specific unit applies.
- `error`: Error information. It is `null` on success.

### 2.2 Releasing Strings

All non-null `char*` values returned by the dynamic library are allocated by the library and must be released by the caller:

```c
void free_string(char* p);
```

Example:

```c
char* ret = read_pos_actual(1);
// parse ret as JSON
free_string(ret);
```

### 2.3 Supported Data Types

The `dataType` parameter of the generic OD integer read/write APIs supports:

```text
Int8, I8
Uint8, U8
Int16, I16
Uint16, U16
Int32, I32
Uint32, U32
```

Note: The `value` parameter of `write_int` is declared as `int32_t` in the C ABI. Before writing a `Uint32/U32` value greater than `INT32_MAX`, confirm how the target device and calling language handle the 32-bit bit pattern.

## 3. Session APIs

### 3.1 create_serial_port

```c
char* create_serial_port(char* portName, int baudRate);
```

Creates a serial communication session for RS232/RS485.

Parameters:

- `portName`: The serial port name, for example, `"COM13"`.
- `baudRate`: The baud rate, for example, `38400`.

Returns:

```json
{ "ok": true, "data": null, "unit": null, "error": null }
```

### 3.2 create_pcan_port

```c
char* create_pcan_port(char* usbBus, char* rate);
```

Creates a PCAN communication session.

Parameters:

- `usbBus`: The PCAN USB bus, for example, `"USB0"` or `"USB1"`.
- `rate`: The baud-rate string. Supported values are `"50K"`, `"125K"`, `"250K"`, `"500K"`, and `"1000K"`.

### 3.3 create_cxcan_port

```c
char* create_cxcan_port(char* canPort, char* rate);
```

Creates a CxCAN communication session.

Parameters:

- `canPort`: The CAN port.
- `rate`: The baud-rate string. Supported values are `"50K"`, `"125K"`, `"250K"`, `"500K"`, and `"1000K"`.

### 3.4 close_session

```c
char* close_session(void);
```

Closes the current communication session.

## 4. Port List API

### 4.1 list_ports

```c
char* list_ports(char* type);
```

Gets the list of available ports.

Parameters:

- `type`: The port type. Supported values are `"SERIAL"`, `"PCAN"`, and `"CXCAN"`.

Successful response:

```json
{
  "ok": true,
  "data": ["COM1", "COM2"],
  "unit": null,
  "error": null
}
```

## 5. Generic OD APIs

### 5.1 read_int

```c
char* read_int(int station, int address, char* dataType);
```

Reads an OD integer value.

Parameters:

- `station`: The device station number.
- `address`: The OD address. The target device's object dictionary or communication protocol defines how the index and subindex are encoded.
- `dataType`: The data-type string.

Successful response:

```json
{ "ok": true, "data": 123, "unit": null, "error": null }
```

### 5.2 write_int

```c
char* write_int(int station, int address, char* dataType, int32_t value);
```

Writes an OD integer value.

The target device determines the valid ranges of `station`, `address`, and `value`. For `Uint32/U32`, also note that the C ABI type of `value` is `int32_t`.

### 5.3 read_string

```c
char* read_string(int station, int address);
```

Reads an OD string.

Successful response:

```json
{ "ok": true, "data": "text", "unit": null, "error": null }
```

### 5.4 write_string

```c
char* write_string(int station, int address, char* data);
```

Writes an OD string.

## 6. P0 APIs

```c
char* read_encoder_ppr(int station);
char* read_pos_actual(int station);
char* read_speed_real(int station);
char* read_torque(int station);
char* set_operation_mode(int station, int mode);
char* set_controlword(int station, int controlword);
char* set_target_position(int station, int32_t position);
char* set_position_p(int station, int32_t p);
char* set_speed_p(int station, int32_t p);
char* set_speed_i(int station, int32_t i);
char* drive_reboot(int station);
char* read_torque_coefficient(int station);
char* set_stiffness(int station, int32_t stiffness);
char* set_damping(int station, int32_t damping);
```

Unit conventions:

| API                       | unit          |
| ------------------------- | ------------- |
| `read_encoder_ppr`        | `null`        |
| `read_pos_actual`         | `deg`         |
| `read_speed_real`         | `deg/s`       |
| `read_torque`             | `Nm`          |
| `read_torque_coefficient` | `Nm/Arms*100` |

## 7. P1 APIs

```c
char* read_damping(int station);
char* read_stiffness(int station);
char* read_operation_mode_buffer(int station);
char* read_statusword(int station);
char* read_iq(int station);
char* set_cmd_q_max(int station, int32_t q_max);
char* read_max_current(int station);
char* set_max_torque(int station, int32_t max_torque);
```

Unit conventions:

| API                          | unit             |
| ---------------------------- | ---------------- |
| `read_damping`               | `kg*m^2/deg/s`   |
| `read_stiffness`             | `kg*m^2/deg/s^2` |
| `read_operation_mode_buffer` | `null`           |
| `read_statusword`            | `null`           |
| `read_iq`                    | `Arms`           |
| `read_max_current`           | `AP`             |

Note: The target device protocol defines the exact meaning of `AP` and the valid parameter ranges of the APIs above. Consult the device manual for the applicable model before use.

## 8. P2 APIs

```c
char* set_profile_speed(int station, int32_t speed);
char* set_profile_acc(int station, int32_t acc);
char* set_profile_dec(int station, int32_t dec);
```

On a successful write, both `data` and `unit` are `null`.

## 9. Diagnostic APIs

### 9.1 get_last_error_msg

```c
int get_last_error_msg(char* buffer, int bufferSize);
```

Gets the most recent error message. The new JSON APIs generally do not require this API.

### 9.2 clear_last_error

```c
void clear_last_error(void);
```

Clears the most recent error message.

## 10. C Usage Example

```c
char* ret = create_serial_port("COM13", 38400);
if (ret != NULL) {
    // parse JSON
    free_string(ret);
}

ret = read_pos_actual(1);
if (ret != NULL) {
    // parse JSON: {"ok":true,"data":90,"unit":"deg","error":null}
    free_string(ret);
}

ret = set_target_position(2, 90);
if (ret != NULL) {
    // parse JSON
    free_string(ret);
}

ret = close_session();
if (ret != NULL) {
    free_string(ret);
}
```

## 11. Key Points for Calling from Python with ctypes

The return type must be set to `ctypes.c_void_p` so that the raw pointer can be passed back to `free_string`.

```python
dll.read_pos_actual.argtypes = [ctypes.c_int]
dll.read_pos_actual.restype = ctypes.c_void_p
dll.free_string.argtypes = [ctypes.c_void_p]
dll.free_string.restype = None

ptr = dll.read_pos_actual(1)
if not ptr:
    raise RuntimeError("native function returned null pointer")
try:
    raw = ctypes.string_at(ptr).decode("utf-8")
    result = json.loads(raw)
finally:
    dll.free_string(ptr)
```
