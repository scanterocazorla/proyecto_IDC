#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <proyecto_IDC_inferencing.h> 

// 1. Configuración WiFi y MQTT
const char* ssid = "iPhone XR";
const char* password = "12345678";
const char* mqtt_server = "172.20.10.2"; 
const char* mqtt_topic = "ventilador/estado";

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;

// 2. Variables para el control del tiempo (Muestreo a 67Hz)
const unsigned long sampling_period_us = 14925; 
unsigned long last_sampling_time = 0;

// Buffer donde guardaremos los datos antes de pasárselos a la IA
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

// Función para copiar los datos del buffer cuando la IA los pida
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void setup() {
    Serial.begin(115200);
    
    // Inicializar I2C
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("Fallo al encontrar el chip MPU6050");
        while (1) delay(10);
    }
    
    // Configurar el rango del acelerómetro a +-2g (igual que en el dataset)
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

    // Conectar a la red WiFi
    setup_wifi();
    client.setServer(mqtt_server, 1883);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // BUCLE DE MUESTREO (No bloqueante, se ejecuta exactamente cada 14.9 ms)
    if (micros() - last_sampling_time >= sampling_period_us) {
        last_sampling_time = micros();

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Convertimos a Fuerzas G (dividiendo por 9.81, ya que Adafruit mide en m/s^2)
        features[feature_ix + 0] = a.acceleration.x / 9.80665;
        features[feature_ix + 1] = a.acceleration.y / 9.80665;
        features[feature_ix + 2] = a.acceleration.z / 9.80665;
        
        feature_ix += 3;

        // El buffer esta lleno?
        if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            
            // Preparamos la estructura de la señal para Edge Impulse
            signal_t signal;
            signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
            signal.get_data = &raw_feature_get_data;

            //ejecutar inferencia
            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            
            if (res == EI_IMPULSE_OK) {
                //Buscamos cuál de las 3 clases tiene mayor probabilidad
                int mejor_clase_idx = 0;
                float max_val = 0;
                
                for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                    if (result.classification[ix].value > max_val) {
                        max_val = result.classification[ix].value;
                        mejor_clase_idx = ix;
                    }
                }

                // Estado detectado en texto
                String estado_detectado = result.classification[mejor_clase_idx].label;
                float confianza = result.classification[mejor_clase_idx].value;

                Serial.print("Estado Detectado: "); Serial.println(estado_detectado);

                //enviar por mqtt en json
                String payload = "{\"estado\":\"" + estado_detectado + "\",\"confianza\":" + String(confianza, 2) + "}";
                client.publish(mqtt_topic, payload.c_str());
            }

            //Reiniciamos el índice del buffer
            feature_ix = 0;
        }
    }
}

void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Conectando a "); Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi conectado. Dirección IP: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Intentando conexión MQTT...");
        // Intentar conectar con un ID de cliente aleatorio
        if (client.connect("PicoW_Ventilador")) {
            Serial.println("¡Conectado al Broker MQTT!");
        } else {
            Serial.print("Fallo, rc="); Serial.print(client.state());
            Serial.println(" Intentando de nuevo en 5 segundos");
            delay(5000);
        }
    }
}