// ==========================================================================================
// Project: Jeeves.Controller
// File:    strings.h
// Version: 0.9
// Date:    2024-03-03
// License: MIT https://github.com/PaoloM/Jeeves.Controller/blob/main/LICENSE
// ==========================================================================================

#define STR_MESSAGE_RECEIVED_FORMAT              "Message arrived on topic [%s]: \"%s\""
#define STR_MQTT_STARTUP_MESSAGE_FORMAT          "Sensor %s starting up"
#define STR_GET_TAG_API_FORMAT                   "/api/things/0x%06X/tag"
#define STR_GET_MQTT_BROKER_IP_API               "/api/mqttserver"
#define STR_STATUS_MESSAGE                       "Status:"
#define STR_STATUS_UNKNOWN                       "Unknown"
#define STR_CMD_TOPIC                            "cmd"
#define STR_CMD_TOPIC_LOG_FORMAT                 "Commands topic:%s"
#define STR_BROKER_LOG_FORMAT                    "Broker:%s"
#define STR_DEFAULT_LOCATION                     "unknown"

#define STR_MQTT_LOG_PREFIX                      "MQTT    "
#define STR_STARTUP_LOG_PREFIX                   "START   "
#define STR_WIFI_LOG_PREFIX                      "WIFI    "
#define STR_WIFIMAN_LOG_PREFIX                   "WIFIMAN "
#define STR_OTA_LOG_PREFIX                       "OTA     "

#define STR_OTA_START_UPDATE_MESSAGE             "Start updating %s"
#define STR_OTA_END_UPDATE_MESSAGE               "End"
#define STR_OTA_UPDATE_PROGRESS_FORMAT           "Progress: %u%%"
#define STR_OTA_ERROR_MESSAGE_FORMAT             "Error[%u]: %s"
#define STR_OTA_ERROR_AUTH_FAILED                "Auth Failed"
#define STR_OTA_ERROR_BEGIN_FAILED               "Begin Failed"
#define STR_OTA_ERROR_CONNECT_FAILED             "Connect Failed"
#define STR_OTA_ERROR_RECEIVE_FAILED             "Receive Failed"
#define STR_OTA_ERROR_END_FAILED                 "End Failed"

#define STR_STARTUP_MESSAGE_FORMAT               "Jeeves.%s starting up"
#define STR_STARTUP_VERSION_MESSAGE_FORMAT       "Firmware v%s"
#define STR_STARTUP_DEVICE_MESSAGE_FORMAT        "Device ID:%06X"

#define STR_SENSOR_TOPIC_TIMESTAMP               "timestamp"
#define STR_SENSOR_TOPIC_DHT_TEMPERATURE         "temperature"
#define STR_SENSOR_TOPIC_DHT_HUMIDITY            "humidity"
#define STR_SENSOR_TOPIC_KY040_VALUE             "position"
#define STR_SENSOR_TOPIC_PMS5003_AE1P0           "airquality/AE1P0"
#define STR_SENSOR_TOPIC_PMS5003_AE2P5           "airquality/AE2P5"
#define STR_SENSOR_TOPIC_PMS5003_AE10P0          "airquality/AE10P0"
#define STR_SENSOR_TOPIC_PMS5003_AQI2P5          "airquality/AQI2P5"
#define STR_SENSOR_TOPIC_PMS5003_AQI2P5LABEL     "airquality/AQI2P5LABEL"
#define STR_SENSOR_TOPIC_PMS5003_PPD0P3          "airquality/PPD0P3"
#define STR_SENSOR_TOPIC_PMS5003_PPD0P5          "airquality/PPD0P5"
#define STR_SENSOR_TOPIC_PMS5003_PPD1P0          "airquality/PPD1P0"
#define STR_SENSOR_TOPIC_PMS5003_PPD2P5          "airquality/PPD2P5"
#define STR_SENSOR_TOPIC_PMS5003_PPD5P0          "airquality/PPD5P0"
#define STR_SENSOR_TOPIC_PMS5003_PPD10P0         "airquality/PPD10P0"