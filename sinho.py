import RPi.GPIO as GPIO
import time
import json
import paho.mqtt.client as mqtt
    
# GPIO 핀 번호 설정
TRIG_PIN = 13
ECHO_PIN = 19
RED_LED_PIN = 23
GREEN_LED_PIN = 24
BUZZER_PIN = 12

# GPIO 핀 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(RED_LED_PIN, GPIO.OUT)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

#pwm 생성
buz_pwm = GPIO.PWM(BUZZER_PIN, 100)

# MQTT 브로커 정보
MQTT_HOST = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_KEEPALIVE_INTERVAL = 60

MQTT_PUB_TOPIC = "mobile/gong/sensing"
MQTT_SUB_TOPIC = "mobile/gong/LED"

def on_message(client, userdata, message):
    result = str(message.payload.decode("utf-8"))
    print(f"received message = {result}")
    value = json.loads(result)
    
    print(value)
    if value.upper() == "ON":
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
        GPIO.output(RED_LED_PIN, GPIO.LOW)
    elif value.upper() == "OFF":
        GPIO.output(GREEN_LED_PIN, GPIO.LOW)
        GPIO.output(RED_LED_PIN, GPIO.HIGH)
    else:
        print("Illegal Argument")
    
client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_HOST, MQTT_PORT,MQTT_KEEPALIVE_INTERVAL)
client.subscribe(MQTT_SUB_TOPIC)
client.loop_start()



# 초음파 감지 함수
def detect_distance():
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(TRIG_PIN, GPIO.LOW)
    
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()
        
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()
        
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    print("Distance : ", distance, "cm")
    
    return distance
GPIO.output(RED_LED_PIN, GPIO.HIGH)
GPIO.output(GREEN_LED_PIN, GPIO.LOW)
try:
    # MQTT 메시지 루프 시작
    client.loop_start()
    
    while True:
        # 초음파 감지
        distance = detect_distance()
        
        if distance < 30:
            time.sleep(10)  # 10초 동안 대기
            
            # 초록 LED 켜기, 빨간 LED 끄기, 부저 울리기
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
            client.publish(MQTT_PUB_TOPIC, "ON")
            GPIO.output(RED_LED_PIN, GPIO.LOW)
            GPIO.output(BUZZER_PIN,GPIO.HIGH)
            buz_pwm.start(1)
            time.sleep(1)  # 1초 동안 부저 울리기
            buz_pwm.stop(10)
            time.sleep(10)  # 초록 LED 유지 시간
            
            
            
            # 10초 후에 2초마다 부저 울리기
            time.sleep(10)
            
            for _ in range(10):
                #p.ChangeFrequency(scale[list[i]])
                #time.sleep(term[i])
                time.sleep(0.5)
                buz_pwm.start(1)
                time.sleep(0.5)
                buz_pwm.stop()
            # 초록 LED 끄기
            GPIO.output(GREEN_LED_PIN, GPIO.LOW)
            client.publish(MQTT_PUB_TOPIC, "OFF")
            
            # 빨간 LED 켜기
            GPIO.output(RED_LED_PIN, GPIO.HIGH)
        
        time.sleep(0.1)  # 0.1초 대기
        
except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()
    GPIO.cleanup()