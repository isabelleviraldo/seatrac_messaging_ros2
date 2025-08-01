#!/usr/bin/env python3
import construct

# ================================================================================================
# Blueprint Subsea SeaTrac Interface Control Document Definitions
# Firmware Version: 2.2

# ================================================================================================
# Enumeration definitions

AMSGTYPE_E = construct.Enum(construct.Int8ul,
    MSG_OWAY=0x00,
    MSG_OWAYU=0x01,
    MSG_REQ=0x02,
    MSG_RESP=0x03,
    MSG_REQU=0x04,
    MSG_RESPU=0x05,
    MSG_REQX=0x06,
    MSG_RESPX=0x07,
    MSG_UNKNOWN=0xFF,
)

APAYLOAD_E = construct.Enum(construct.Int8ul,
    PLOAD_PING=0x00,
    PLOAD_ECHO=0x01,
    PLOAD_NAV=0x02,
    PLOAD_DAT=0x03,
    PLOAD_DEX=0x04,
)

BAUDRATE_E = construct.Enum(construct.Int8ul,
    BAUD_4800=0x07,
    BAUD_9600=0x08,
    BAUD_14400=0x09,
    BAUD_19200=0x0A,
    BAUD_38400=0x0B,
    BAUD_57600=0x0C,
    BAUD_115200=0x0D,
)

BID_E = construct.Enum(construct.Int8ul,
    BEACON_ALL=0x00,
    BEACON_ID_1=0x01,
    BEACON_ID_2=0x02,
    BEACON_ID_3=0x03,
    BEACON_ID_4=0x04,
    BEACON_ID_5=0x05,
    BEACON_ID_6=0x06,
    BEACON_ID_7=0x07,
    BEACON_ID_8=0x08,
    BEACON_ID_9=0x09,
    BEACON_ID_10=0x0A,
    BEACON_ID_11=0x0B,
    BEACON_ID_12=0x0C,
    BEACON_ID_13=0x0D,
    BEACON_ID_14=0x0E,
    BEACON_ID_15=0x0F,
)

CAL_ACTION_E = construct.Enum(construct.Int8ul,
    CAL_ACC_DEFAULTS=0x00,
    CAL_ACC_RESET=0x01,
    CAL_ACC_CALC=0x02,
    CAL_MAG_DEFAULTS=0x03,
    CAL_MAG_RESET=0x04,
    CAL_MAG_CALC=0x05,
    CAL_PRES_OFFSET_RESET=0x06,
    CAL_PRES_OFFSET_CALC=0x07,
)

# CID_E are defined with command definitions below

CST_E = construct.Enum(construct.Int8ul,
    CST_OK=0x00,
    CST_FAIL=0x01,
    CST_EEPROM_ERROR=0x03,
    CST_CMD_PARAM_MISSING=0x04,
    CST_CMD_PARAM_INVALID=0x05,
    CST_PROG_FLASH_ERROR=0x0A,
    CST_PROG_FIRMWARE_ERROR=0x0B,
    CST_PROG_SECTION_ERROR=0x0C,
    CST_PROG_LENGTH_ERROR=0x0D,
    CST_PROG_DATA_ERROR=0x0E,
    CST_PROG_CHECKSUM_ERROR=0x0F,
    CST_XCVR_BUSY=0x30,
    CST_XCVR_ID_REJECTED=0x31,
    CST_XCVR_CSUM_ERROR=0x32,
    CST_XCVR_LENGTH_ERROR=0x33,
    CST_XCVR_RESP_TIMEOUT=0x34,
    CST_XCVR_RESP_ERROR=0x35,
    CST_XCVR_RESP_WRONG=0x36,
    CST_XCVR_PLOAD_ERROR=0x37,
    CST_XCVR_STATE_STOPPED=0x3A,
    CST_XCVR_STATE_IDLE=0x3B,
    CST_XCVR_STATE_TX=0x3C,
    CST_XCVR_STATE_REQ=0x3D,
    CST_XCVR_STATE_RX=0x3E,
    CST_XCVR_STATE_RESP=0x3F,
    CST_DEX_SOCKET_ERROR=0x70,
    CST_DEX_RX_SYNC=0x71,
    CST_DEX_RX_DATA=0x72,
    CST_DEV_RX_SEQ_ERROR=0x73,
    CST_DEX_RX_MSG_ERROR=0x74,
    CST_DEX_REQ_ERROR=0x75,
    CST_DEX_RESP_TMO_ERROR=0x76,
    CST_DEX_RESP_MSG_ERROR=0x77,
    CST_DEX_RESP_REMOTE_ERROR=0x78,
)

STATUSMODE_E = construct.Enum(construct.Int8ul,
    STATUS_MODE_MANUAL=0x00,
    STATUS_MODE_1HZ=0x01,
    STATUS_MODE_2HZ5=0x02,
    STATUS_MODE_5HZ=0x03,
    STATUS_MODE_10HZ=0x04,
    STATUS_MODE_25HZ=0x05,
)

SETTINGS_STATUS_MODE_E = construct.Enum(construct.BitsInteger(3),
    STATUS_MODE_MANUAL=0x0,
    STATUS_MODE_1HZ=0x1,
    STATUS_MODE_2HZ5=0x2,
    STATUS_MODE_5HZ=0x3,
    STATUS_MODE_10HZ=0x4,
    STATUS_MODE_25HZ=0x5,
)

# ================================================================================================
# Type definitions

ACOMSG_T = construct.Struct(
    'MSG_DEST_ID' / BID_E,
    'MSG_SRC_ID' / BID_E,
    'MSG_TYPE' / AMSGTYPE_E,
    'MSG_DEPTH' / construct.Int16ul,
    'MSG_PAYLOAD_ID' / APAYLOAD_E,
    'MSG_PAYLOAD_LEN' / construct.Int8ul,
    'MSG_PAYLOAD' / construct.Array(lambda this: this.MSG_PAYLOAD_LEN, construct.Int8ul),
)

ACOFIX_RANGE_T = construct.Struct(
    'RANGE_COUNT' / construct.Int32ul,
    'RANGE_TIME' / construct.Int32sl,
    'RANGE_DIST' / construct.Int16ul,
)

ACOFIX_USBL_T = construct.Struct(
    'USBL_CHANNELS' / construct.Int8ul,
    'USBL_RSSI' / construct.Array(lambda this: this.USBL_CHANNELS, construct.Int16sl),
    'USBL_AZIMUTH' / construct.Int16sl,
    'USBL_ELEVATION' / construct.Int16sl,
    'USBL_FIT_ERROR' / construct.Int16sl,
)

ACOFIX_POSITION_T = construct.Struct(
    'POSITION_EASTING' / construct.Int16sl,
    'POSITION_NORTHING' / construct.Int16sl,
    'POSITION_DEPTH' / construct.Int16sl,
)

ACOFIX_T = construct.Struct(
    'DEST_ID' / BID_E,
    'SRC_ID' / BID_E,
    'FLAGS' / construct.BitStruct(
        'RESERVED' / construct.Padding(3),
        'POSITION_FLT_ERROR' / construct.Flag,
        'POSITION_ENHANCED' / construct.Flag,
        'POSITION_VALID' / construct.Flag,
        'USBL_VALID' / construct.Flag,
        'RANGE_VALID' / construct.Flag,
    ),
    'MSG_TYPE' / AMSGTYPE_E,
    'ATTITUDE_YAW' / construct.Int16sl,
    'ATTITUDE_PITCH' / construct.Int16sl,
    'ATTITUDE_ROLL' / construct.Int16sl,
    'DEPTH_LOCAL' / construct.Int16ul,
    'VOS' / construct.Int16ul,
    'RSSI' / construct.Int16sl,
    'RANGE' / construct.If(lambda this: this.FLAGS.RANGE_VALID, ACOFIX_RANGE_T),
    'USBL' / construct.If(lambda this: this.FLAGS.USBL_VALID, ACOFIX_USBL_T),
    'POSITION' / construct.If(lambda this: this.FLAGS.POSITION_VALID, ACOFIX_POSITION_T),
)

AHRSCAL_T = construct.Struct(
    'ACC_MIN_X' / construct.Default(construct.Int16sl, -270),
    'ACC_MIN_Y' / construct.Default(construct.Int16sl, -270),
    'ACC_MIN_Z' / construct.Default(construct.Int16sl, -270),
    'ACC_MAX_X' / construct.Default(construct.Int16sl, 270),
    'ACC_MAX_Y' / construct.Default(construct.Int16sl, 270),
    'ACC_MAX_Z' / construct.Default(construct.Int16sl, 270),
    'MAG_VALID' / construct.Default(construct.Flag, True),
    'MAG_HARD_X' / construct.Default(construct.Float32l, 0),
    'MAG_HARD_Y' / construct.Default(construct.Float32l, 0),
    'MAG_HARD_Z' / construct.Default(construct.Float32l, 0),
    'MAG_SOFT_X' / construct.Default(construct.Float32l, 1),
    'MAG_SOFT_Y' / construct.Default(construct.Float32l, 1),
    'MAG_SOFT_Z' / construct.Default(construct.Float32l, 1),
    'MAG_FIELD' / construct.Default(construct.Float32l, 0),
    'MAG_ERROR' / construct.Default(construct.Float32l, 100.0),
    'GYRO_OFFSET_X' / construct.Default(construct.Int16sl, 0),
    'GYRO_OFFSET_Y' / construct.Default(construct.Int16sl, 0),
    'GYRO_OFFSET_Z' / construct.Default(construct.Int16sl, 0),
)

FIRMWARE_T = construct.Struct(
    'VALID' / construct.Flag,
    'PART_NUMBER' / construct.Int16ul,
    'VERSION_MAJ' / construct.Int8ul,
    'VERSION_MIN' / construct.Int8ul,
    'VERSION_BUILD' / construct.Int16ul,
    'CHECKSUM' / construct.Int32ul,
)

HARDWARE_T = construct.Struct(
    'PART_NUMBER' / construct.Int16ul,
    'PART_REV' / construct.Int8ul,
    'SERIAL_NUMBER' / construct.Int32ul,
    'FLAGS_SYS' / construct.Int16ul,
    'FLAGS_USR' / construct.Int16ul,
)

IPADDR_T = construct.Array(4, construct.Int8ul)

MACADDR_T = construct.Array(6, construct.Int8ul)

NAV_QUERY_T = construct.BitStruct(
    'QRY_DATA' / construct.Flag,
    'RESERVED' / construct.Padding(3),
    'QRY_ATTITUDE' / construct.Flag,
    'QRY_TEMP' / construct.Flag,
    'QRY_SUPPLY' / construct.Flag,
    'QRY_DEPTH' / construct.Flag,
)

STATUS_BITS_T = construct.BitStruct(
    'RESERVED' / construct.Padding(2),
    'AHRS_COMP_DATA' / construct.Flag,
    'AHRS_RAW_DATA' / construct.Flag,
    'ACC_CAL' / construct.Flag,
    'MAG_CAL' / construct.Flag,
    'ATTITUDE' / construct.Flag,
    'ENVIRONMENT' / construct.Flag,
)

SETTINGS_T = construct.Struct(
    'STATUS_FLAGS' / construct.BitStruct(
        'RESERVED' / construct.Padding(5),
        'STATUS_MODE' / SETTINGS_STATUS_MODE_E,
    ),
    'STATUS_OUTPUT' / STATUS_BITS_T,
    'UART_MAIN_BAUD' / BAUDRATE_E,
    'UART_AUX_BAUD' / construct.Default(BAUDRATE_E, 0x0D), # (Reserved, use default of `BAUD_115200`)
    'NET_MAC_ADDR' / construct.Default(MACADDR_T, [0, 0, 0, 0, 0, 0]), # (Reserved, use default of `0`)
    'NET_IP_ADDR' / construct.Default(IPADDR_T, [192, 168, 1, 250]), # (Reserved, use default of `0xC0A801FA`)
    'NET_IP_SUBNET' / construct.Default(IPADDR_T, [255, 255, 0, 0]), # (Reserved, use default of `0xFFFF0000`)
    'NET_IP_GATEWAY' / construct.Default(IPADDR_T, [192, 168, 1, 1]), # (Reserved, use default of `0xC0A80101`)
    'NET_IP_DNS' / construct.Default(IPADDR_T, [192, 168, 1, 1]), # (Reserved, use default of `0xC0A80101`)
    'NET_TCP_PORT' / construct.Default(construct.Int16ul, 8100), # (Reserved, use default of `8100`)
    'ENV_FLAGS' / construct.BitStruct(
        'RESERVED' / construct.Padding(6),
        'AUTO_PRESSURE_OFS' / construct.Flag,
        'AUTO_VOS' / construct.Flag,
    ),
    'ENV_PRESSURE_OFS' / construct.Int32sl,
    'ENV_SALINITY' / construct.Int16ul,
    'ENV_VOS' / construct.Int16ul,
    'AHRS_FLAGS' / construct.BitStruct(
        'RESERVED' / construct.Padding(7),
        'AUTO_CAL_MAG' / construct.Flag,
    ),
    'AHRS_CAL' / AHRSCAL_T,
    'AHRS_YAW_OFS' / construct.Int16ul,
    'AHRS_PITCH_OFS' / construct.Int16ul,
    'AHRS_ROLL_OFS' / construct.Int16ul,
    'XCVR_FLAGS' / construct.BitStruct(
        'XCVR_DIAG_MSGS' / construct.Flag,
        'XCVR_FIX_MSGS' / construct.Flag,
        'XCVR_USBL_MSGS' / construct.Flag,
        'RESERVED' / construct.Padding(3),
        'XCVR_POSFLT_ENABLE' / construct.Flag,
        'USBL_USE_AHRS' / construct.Flag,
    ),
    'XCVR_BEACON_ID' / BID_E,
    'XCVR_RANGE_TMO' / construct.Int16ul,
    'XCVR_RESP_TIME' / construct.Int16ul,
    'XCVR_YAW' / construct.Int16ul,
    'XCVR_PITCH' / construct.Int16ul,
    'XCVR_ROLL' / construct.Int16ul,
    'XCVR_POSFLT_VEL' / construct.Int8ul,
    'XCVR_POSFLT_ANG' / construct.Int8ul,
    'XCVR_POSFLT_TMO' / construct.Int8ul,
)

CID_STATUS_ENVIRONMENT_T = construct.Struct(
    'ENV_SUPPLY' / construct.Int16ul,
    'ENV_TEMP' / construct.Int16sl,
    'ENV_PRESSURE' / construct.Int32sl,
    'ENV_DEPTH' / construct.Int32sl,
    'ENV_VOS' / construct.Int16ul,
)

CID_STATUS_ATTITUDE_T = construct.Struct(
    'ATT_YAW' / construct.Int16sl,
    'ATT_PITCH' / construct.Int16sl,
    'ATT_ROLL' / construct.Int16sl,
)

CID_STATUS_MAG_CAL_T = construct.Struct(
    'MAG_CAL_BUF' / construct.Int8ul,
    'MAG_CAL_VALID' / construct.Flag,
    'MAG_CAL_AGE' / construct.Int32ul,
    'MAG_CAL_FIT' / construct.Int8ul,
)

CID_STATUS_ACC_CAL_T = construct.Struct(
    'ACC_LIM_MIN_X' / construct.Int16sl,
    'ACC_LIM_MIN_Y' / construct.Int16sl,
    'ACC_LIM_MIN_Z' / construct.Int16sl,
    'ACC_LIM_MAX_X' / construct.Int16sl,
    'ACC_LIM_MAX_Y' / construct.Int16sl,
    'ACC_LIM_MAX_Z' / construct.Int16sl,
)

CID_STATUS_AHRS_RAW_DATA_T = construct.Struct(
    'AHRS_RAW_ACC_X' / construct.Int16sl,
    'AHRS_RAW_ACC_Y' / construct.Int16sl,
    'AHRS_RAW_ACC_Z' / construct.Int16sl,
    'AHRS_RAW_MAG_X' / construct.Int16sl,
    'AHRS_RAW_MAG_Y' / construct.Int16sl,
    'AHRS_RAW_MAG_Z' / construct.Int16sl,
    'AHRS_RAW_GYRO_X' / construct.Int16sl,
    'AHRS_RAW_GYRO_Y' / construct.Int16sl,
    'AHRS_RAW_GYRO_Z' / construct.Int16sl,
)

CID_STATUS_AHRS_COMP_DATA_T = construct.Struct(
    'AHRS_COMP_ACC_X' / construct.Float32l,
    'AHRS_COMP_ACC_Y' / construct.Float32l,
    'AHRS_COMP_ACC_Z' / construct.Float32l,
    'AHRS_COMP_MAG_X' / construct.Float32l,
    'AHRS_COMP_MAG_Y' / construct.Float32l,
    'AHRS_COMP_MAG_Z' / construct.Float32l,
    'AHRS_COMP_GYRO_X' / construct.Float32l,
    'AHRS_COMP_GYRO_Y' / construct.Float32l,
    'AHRS_COMP_GYRO_Z' / construct.Float32l,
)

# ================================================================================================
# Command ID command and request definitions

CID_SYS_ALIVE = {
    'cid': 0x01,
    'command': construct.Struct(),
    'response': construct.Struct(
        'SECONDS' / construct.Int32ul,
    ),
}

CID_SYS_INFO = {
    'cid': 0x02,
    'command': construct.Struct(),
    'response': construct.Struct(
        'SECONDS' / construct.Int32ul,
        'SECTION' / construct.Enum(construct.Int8ul,
            BOOTLOADER_APPLICATION=0,
            MAIN_APPLICATION=1
        ),
        'HARDWARE' / HARDWARE_T,
        'BOOT_FIRMWARE' / FIRMWARE_T,
        'MAIN_FIRMWARE' / FIRMWARE_T,
        'BOARD_REV' / construct.Int8ul,
    ),
}

CID_SYS_REBOOT = {
    'cid': 0x03,
    'command': construct.Struct(
        'CHECK' / construct.Const(0x6A95, construct.Int16ul),
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

# CID_SYS_ENGINEERING is unimplemented by design

CID_PROG_INIT = {
    'cid': 0x0D,
    'command': construct.Struct(
        'FW_SECTION' / construct.Int8ul,
        'FW_PART_NUMBER' / construct.Int16ul,
        'FW_PART_REV_MIN' / construct.Int8ul,
        'FW_PART_REV_MAX' / construct.Int8ul,
        'FW_SERIAL_NUMBER' / construct.Int32ul,
        'FW_DATA_FORMAT' / construct.Int8ul,
        'FW_LENGTH' / construct.Int32ul,
        'FW_CHECKSUM' / construct.Int32ul,
        'FW_SIGNATURE' / construct.Array(20, construct.Int8ul),
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_PROG_BLOCK = {
    'cid': 0x0E,
    'command': construct.Struct(
        'DATA_LENGTH' / construct.Int16ul,
        'DATA' / construct.Array(lambda this: this.DATA_LENGTH, construct.Int8ul),
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_PROG_UPDATE = {
    'cid': 0x0F,
    'command': construct.Struct(),
    'response': construct.Struct(
        'STATUS' / CST_E,
    )
}

CID_STATUS = {
    'cid': 0x10,
    'command': construct.Struct(
        'STATUS_OUTPUT' / STATUS_BITS_T,
    ),
    'response': construct.Struct(
        'STATUS_OUTPUT' / STATUS_BITS_T,
        'TIMESTAMP' / construct.Int64ul,
        'ENVIRONMENT' / construct.If(lambda this: this.STATUS_OUTPUT.ENVIRONMENT, CID_STATUS_ENVIRONMENT_T),
        'ATTITUDE' / construct.If(lambda this: this.STATUS_OUTPUT.ATTITUDE, CID_STATUS_ATTITUDE_T),
        'MAG_CAL' / construct.If(lambda this: this.STATUS_OUTPUT.MAG_CAL, CID_STATUS_MAG_CAL_T),
        'ACC_CAL' / construct.If(lambda this: this.STATUS_OUTPUT.ACC_CAL, CID_STATUS_ACC_CAL_T),
        'AHRS_RAW_DATA' / construct.If(lambda this: this.STATUS_OUTPUT.AHRS_RAW_DATA, CID_STATUS_AHRS_RAW_DATA_T),
        'AHRS_COMP_DATA' / construct.If(lambda this: this.STATUS_OUTPUT.AHRS_COMP_DATA, CID_STATUS_AHRS_COMP_DATA_T),
    ),
}

CID_STATUS_CFG_GET = {
    'cid': 0x11,
    'command': construct.Struct(),
    'response': construct.Struct(
        'STATUS_OUTPUT' / STATUS_BITS_T,
        'STATUS_MODE' / STATUSMODE_E,
    ),
}

CID_STATUS_CFG_SET = {
    'cid': 0x12,
    'command': construct.Struct(
        'STATUS_OUTPUT' / STATUS_BITS_T,
        'STATUS_MODE' / STATUSMODE_E,
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_SETTINGS_GET = {
    'cid': 0x15,
    'command': construct.Struct(),
    'response': construct.Struct(
        'SETTINGS' / SETTINGS_T,
    ),
}

CID_SETTING_SET = {
    'cid': 0x16,
    'command': construct.Struct(
        'SETTINGS' / SETTINGS_T,
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_SETTINGS_LOAD = {
    'cid': 0x17,
    'command': construct.Struct(),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_SETTINGS_SAVE = {
    'cid': 0x18,
    'command': construct.Struct(),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_SETTINGS_RESET = {
    'cid': 0x19,
    'command': construct.Struct(),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_CAL_ACTION = {
    'cid': 0x20,
    'command': construct.Struct(
        'ACTION' / CAL_ACTION_E,
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_AHRS_CAL_GET = {
    'cid': 0x21,
    'command': construct.Struct(),
    'response': construct.Struct(
        'AHRS_CAL' / AHRSCAL_T,
    ),
}

CID_AHRS_CAL_SET = {
    'cid': 0x22,
    'command': construct.Struct(
        'AHRS_CAL' / AHRSCAL_T,
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_XCVR_ANALYSE = {
    'cid': 0x30,
    'command': construct.Struct(),
    'response': construct.Struct(
        'STATUS' / CST_E,
        'ADC_MEAN' / construct.Int16sl,
        'ADC_PKPK' / construct.Int16ul,
        'ADC_RMS' / construct.Int32ul,
        'RX_LEVEL_PKPK' / construct.Int16sl,
        'RX_LEVEL_RMS' / construct.Int16sl,
    ),
}

CID_XCVR_TX_MSG = {
    'cid': 0x31,
    # TODO
}

CID_XCVR_RX_ERR = {
    'cid': 0x32,
    # TODO
}

CID_XCVR_RX_MSG = {
    'cid': 0x33,
    # TODO
}

CID_XCVR_RX_REQ = {
    'cid': 0x34,
    # TODO
}

CID_XCVR_RX_RESP = {
    'cid': 0x35,
    # TODO
}

CID_XCVR_RX_UNHANDLED = {
    'cid': 0x37,
    # TODO
}

CID_XCVR_USBL = {
    'cid': 0x38,
    # TODO
}

CID_XCVR_FIX = {
    'cid': 0x39,
    'status': construct.Struct(
        'ACO_FIX' / ACOFIX_T,
    ),
}

CID_XCVR_STATUS = {
    'cid': 0x3A,
    'command': construct.Struct(),
    'response': construct.Struct(
        'STATUS' / CST_E,
    ),
}

CID_PING_SEND = {
    'cid': 0x40,
    'command': construct.Struct(
        'DEST_ID' / BID_E,
        'MSG_TYPE' / AMSGTYPE_E,
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
        'BEACON_ID' / BID_E,
    ),
}

CID_PING_REQ = {
    'cid': 0x41,
    # Triggered on ping request
    'status': construct.Struct(
        'ACO_FIX' / ACOFIX_T,
    ),
}

CID_PING_RESP = {
    'cid': 0x42,
    # Triggered on ping response reception
    'status': construct.Struct(
        'ACO_FIX' / ACOFIX_T,
    ),
}

CID_PING_ERROR = {
    'cid': 0x43,
    # Triggered on unsuccessful ping
    'status': construct.Struct(
        'STATUS' / CST_E,
        'BEACON_ID' / BID_E,
    ),
}

CID_ECHO_SEND = {
    'cid': 0x48,
    # TODO
}

CID_ECHO_REQ = {
    'cid': 0x49,
    # TODO
}

CID_ECHO_RESP = {
    'cid': 0x4A,
    # TODO
}

CID_ECHO_ERROR = {
    'cid': 0x4B,
    # TODO
}

CID_NAV_QUERY_SEND = {
    'cid': 0x50,
    'command': construct.Struct(
        'DEST_ID' / BID_E,
        'QUERY_FLAGS' / NAV_QUERY_T,
        'PACKET_LEN' / construct.Int8ul,
        'PACKET_DATA' / construct.Array(lambda this: this.PACKET_LEN, construct.Int8ul),
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
        'DEST_ID' / BID_E,
    ),
}

CID_NAV_QUERY_REQ = {
    'cid': 0x51,
    # Triggered on navigation query response request
    'status': construct.Struct(
        'ACO_FIX' / ACOFIX_T,
        'QUERY_FLAGS' / NAV_QUERY_T,
        'PACKET_LEN' / construct.Int8ul,
        'PACKET_DATA' / construct.Array(lambda this: this.PACKET_LEN, construct.Int8ul),
        'LOCAL_FLAG' / construct.Flag,
    ),
}

CID_NAV_QUERY_RESP = {
    'cid': 0x52,
    # Triggered on navigation query response reception
    'status': construct.Struct(
        'ACO_FIX' / ACOFIX_T,
        'QUERY_FLAGS' / NAV_QUERY_T,
        'REMOTE_DEPTH' / construct.If(lambda this: this.QUERY_FLAGS.QRY_DEPTH, construct.Int32sl),
        'REMOTE_SUPPLY' / construct.If(lambda this: this.QUERY_FLAGS.QRY_SUPPLY, construct.Int16ul),
        'REMOTE_TEMP' / construct.If(lambda this: this.QUERY_FLAGS.QRY_TEMP, construct.Int16sl),
        'REMOTE_YAW' / construct.If(lambda this: this.QUERY_FLAGS.QRY_ATTITUDE, construct.Int16sl),
        'REMOTE_PITCH' / construct.If(lambda this: this.QUERY_FLAGS.QRY_ATTITUDE, construct.Int16sl),
        'REMOTE_ROLL' / construct.If(lambda this: this.QUERY_FLAGS.QRY_ATTITUDE, construct.Int16sl),
        'PACKET_LEN' / construct.If(lambda this: this.QUERY_FLAGS.QRY_DATA, construct.Int8ul),
        'PACKET_DATA' / construct.If(lambda this: this.QUERY_FLAGS.QRY_DATA, construct.Array(lambda this: this.PACKET_LEN, construct.Int8ul)),
        'LOCAL_FLAG' / construct.Flag,
    ),
}

CID_NAV_ERROR = {
    'cid': 0x53,
    # Triggered on unsuccessful navigation query
    'status': construct.Struct(
        'STATUS' / CST_E,
        'BEACON_ID' / BID_E,
    ),
}

CID_NAV_QUEUE_SET = {
    'cid': 0x58,
    # TODO
}

CID_NAV_QUEUE_CLR = {
    'cid': 0x59,
    # TODO
}

CID_NAV_QUEUE_STATUS = {
    'cid': 0x5A,
    # TODO
}

CID_NAV_STATUS_SEND = {
    'cid': 0x5B,
    'command': construct.Struct(
        'BEACON_ID' / BID_E,
        'PACKET_LEN' / construct.Int8ul,
        'PACKET_DATA' / construct.Array(lambda this: this.PACKET_LEN, construct.Int8ul),
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
        'BEACON_ID' / BID_E,
    ),
}

CID_NAV_STATUS_RECEIVE = {
    'cid': 0x5C,
    # Triggered on reception of a navigation status send message
    'status': construct.Struct(
        'ACO_FIX' / ACOFIX_T,
        'BEACON_ID' / BID_E,
        'PACKET_LEN' / construct.Int8ul,
        'PACKET_DATA' / construct.Array(lambda this: this.PACKET_LEN, construct.Int8ul),
        'LOCAL_FLAG' / construct.Flag,
    ),
}

CID_DAT_SEND = {
    'cid': 0x60,
    'command': construct.Struct(
        'DEST_ID' / BID_E,
        'MSG_TYPE' / AMSGTYPE_E,
        'PACKET_LEN' / construct.Int8ul,
        'PACKET_DATA' / construct.Array(lambda this: this.PACKET_LEN, construct.Int8ul),
    ),
    'response': construct.Struct(
        'STATUS' / CST_E,
        'BEACON_ID' / BID_E,
    ),
}

CID_DAT_RECEIVE = {
    'cid': 0x61,
    # Triggered on reception of an incoming data packet
    'status': construct.Struct(
        'ACO_FIX' / ACOFIX_T,
        'ACK_FLAG' / construct.Flag,
        'PACKET_LEN' / construct.Int8ul,
        'PACKET_DATA' / construct.Array(lambda this: this.PACKET_LEN, construct.Int8ul),
        'LOCAL_FLAG' / construct.Flag,
    ),
}

CID_DAT_ERROR = {
    'cid': 0x63,
    'status': construct.Struct(
        'STATUS' / CST_E,
        'BEACON_ID' / BID_E,
    ),
}

CID_DAT_QUEUE_SET = {
    'cid': 0x64,
    # TODO
}

CID_DAT_QUEUE_CLR = {
    'cid': 0x65,
    # TODO
}

CID_DAT_QUEUE_STATUS = {
    'cid': 0x66,
    # TODO
}


if __name__ == '__main__':
    pass