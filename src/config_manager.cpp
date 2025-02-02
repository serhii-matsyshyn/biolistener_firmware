// Config manager is based on WifiManager library (https://github.com/tzapu/WiFiManager/) and it's example code,
// to configure WiFi and parameters for the BioListener device.

#include "config_manager.h"

IPAddressParameter::IPAddressParameter(const char *id, const char *placeholder,
                                       uint32_t address_raw)
    : WiFiManagerParameter("") {
  IPAddress address(address_raw);
  init(id, placeholder, address.toString().c_str(), 16, "", WFM_LABEL_BEFORE);
}

uint32_t IPAddressParameter::getValue() {
  ip.fromString(WiFiManagerParameter::getValue());
  return ip;
}

void IPAddressParameter::setValue(uint32_t address_raw) {
  ip = IPAddress(address_raw);
  WiFiManagerParameter::setValue(ip.toString().c_str(), 16);
}

IntParameter::IntParameter(const char *id, const char *placeholder, long value)
    : WiFiManagerParameter("") {
  init(id, placeholder, String(value).c_str(), 10, "", WFM_LABEL_BEFORE);
}

long IntParameter::getValue() {
  return String(WiFiManagerParameter::getValue()).toInt();
}

void IntParameter::setValue(long value) {
  WiFiManagerParameter::setValue(String(value).c_str(), 10);
}

IntParameter param_port("port", "Brainflow Server Port", 0);
IPAddressParameter param_ip("server", "Brainflow Server IP", 0);

ConfigManager::ConfigManager()
    : shouldSaveConfig(false), param_server_ip(0), param_server_port(0),
      shouldProcessConfigPortal(true) {}

void ConfigManager::begin() {
  Serial.println("Mounting FS...");
  if (LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("Mounted file system");
    loadConfig();
  } else {
    Serial.println("Failed to mount FS");
  }

  Serial.println("Starting config portal");
  Serial.println("Param server ip: " + String(param_server_ip));
  Serial.println("Param server port: " + String(param_server_port));

  wifiManager.setSaveConfigCallback([&]() { saveConfigCallback(); });
  wifiManager.setSaveParamsCallback([&]() { saveConfigCallback(); });
  wifiManager.addParameter(&param_port);
  wifiManager.addParameter(&param_ip);
  wifiManager.setHostname("biolistener");

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wifiManager.setMenu(menu);
  wifiManager.setClass("invert"); // dark theme
  wifiManager.setAPClientCheck(true);

  if (wifiManager.autoConnect(wifiNameWithChipId.c_str())) {
    Serial.println("Connected to WiFi");
  } else {
    Serial.println("Failed to connect to WiFi!");
  }

  Serial.println("Starting config portal for params configuration");

  wifiManager.setConfigPortalTimeout(120);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.startConfigPortal(wifiNameWithChipId.c_str());
}

void ConfigManager::loadConfig() {
  if (LittleFS.exists(configFilePath)) {
    Serial.println("Reading config file");
    File configFile = LittleFS.open(configFilePath, "r");
    if (configFile) {
      Serial.println("Opened config file");
      size_t size = configFile.size();
      std::unique_ptr<char[]> buf(new char[size]);
      configFile.readBytes(buf.get(), size);
      DynamicJsonDocument json(1024);
      auto deserializeError = deserializeJson(json, buf.get());
      if (!deserializeError) {
        Serial.println("Parsed json");
        param_server_ip = json["param_server_ip"].as<uint32_t>();
        param_server_port = json["param_server_port"].as<int>();

        param_port.setValue(param_server_port);
        param_ip.setValue(param_server_ip);
      } else {
        Serial.println("Failed to load json config");
      }
      configFile.close();
    }
  }
}

void ConfigManager::saveConfig() {
  Serial.println("Saving config");
  DynamicJsonDocument json(1024);
  json["param_server_ip"] = param_server_ip;
  json["param_server_port"] = param_server_port;

  File configFile = LittleFS.open(configFilePath, "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return;
  }
  serializeJson(json, configFile);
  configFile.close();
}

void ConfigManager::saveConfigCallback() {
  Serial.println("Saving config (custom params)");
  param_server_ip = param_ip.getValue();
  param_server_port = param_port.getValue();
  saveConfig();
}

// Runs from beginning. When shouldProcessConfigPortal is false, it will stop and never run again
void ConfigManager::configPortalTask(void *parameter) {
  while (shouldProcessConfigPortal) {
    wifiManager.process();
    delay(1);
  }

  Serial.println("Config portal task ended. Stopping config portal.");
  wifiManager.stopConfigPortal();
  vTaskDelete(NULL);
}

// Stop the config portal task (soft stop)
void ConfigManager::stopConfigPortalTask() {
  shouldProcessConfigPortal = false;
}