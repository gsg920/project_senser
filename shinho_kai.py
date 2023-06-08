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

GPIO.output(RED_LED_PIN, GPIO.HIGH)
GPIO.output(GREEN_LED_PIN, GPIO.LOW)

onn = 0

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
    distance = pulse_duration * 17000
    distance = round(distance, 2)
    
    print("Distance : ", distance, "cm")
    
    return distance

def on_message(client, userdata, message):
    global onn
    result = str(message.payload.decode("utf-8"))
    print(f"received message = {result}")
    value = json.loads(result)
    
    print(value)
    if value.upper() == "ON":
        onn = 1

    elif value.upper() == "OFF":
        GPIO.output(GREEN_LED_PIN, GPIO.LOW)
        GPIO.output(RED_LED_PIN, GPIO.HIGH)
        return
    else:
        print("Illegal Argument")

client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_HOST, MQTT_PORT,MQTT_KEEPALIVE_INTERVAL)
client.subscribe(MQTT_SUB_TOPIC)
client.loop_start()

try:
    # MQTT 메시지 루프 시작
    client.loop_start()
    
    while True:
        # 초음파 감지
        distance = detect_distance()
        onni = onn
        if distance < 30:  #초음파 센서 인식 범위 
            time.sleep(10)  # 10초 동안 대기
            
            # 초록 LED 켜기, 빨간 LED 끄기, 부저 울리기
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
            client.publish(MQTT_PUB_TOPIC, "ON")
            GPIO.output(RED_LED_PIN, GPIO.LOW)
            GPIO.output(BUZZER_PIN,GPIO.HIGH)
            buz_pwm.start(1)
            time.sleep(1)  # 1초 동안 부저 울리기
            buz_pwm.stop(1)
            time.sleep(5) #부저 울리기까지 걸리는 시간
            
            
            
            for _ in range(5): #부저 울리는 시간
                #p.ChangeFrequency(scale[list[i]])
                #time.sleep(term[i])
                time.sleep(0.5)
                buz_pwm.start(1)
                time.sleep(0.5)
                buz_pwm.stop(1)
            # 초록 LED 끄기
            GPIO.output(GREEN_LED_PIN, GPIO.LOW)
            client.publish(MQTT_PUB_TOPIC, "OFF")
            
            # 빨간 LED 켜기
            GPIO.output(RED_LED_PIN, GPIO.HIGH)
        
        elif onni > 0:
            # 초록 LED 켜기, 빨간 LED 끄기, 부저 울리기
            GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
            client.publish(MQTT_PUB_TOPIC, "ON")
            GPIO.output(RED_LED_PIN, GPIO.LOW)
            GPIO.output(BUZZER_PIN,GPIO.HIGH)
            buz_pwm.start(1)
            time.sleep(1)  # 1초 동안 부저 울리기
            buz_pwm.stop(1)
            time.sleep(5) #부저 울리기까지 걸리는 시간
            
            
            
            for _ in range(5): #부저 울리는 시간
                #p.ChangeFrequency(scale[list[i]])
                #time.sleep(term[i])
                time.sleep(0.5)
                buz_pwm.start(1)
                time.sleep(0.5)
                buz_pwm.stop(1)
            # 초록 LED 끄기
            GPIO.output(GREEN_LED_PIN, GPIO.LOW)
            client.publish(MQTT_PUB_TOPIC, "OFF")
            
            # 빨간 LED 켜기
            GPIO.output(RED_LED_PIN, GPIO.HIGH)
            onn = 0
            
        time.sleep(0.1)  # 0.1초 대기


except KeyboardInterrupt:
    client.loop_stop()
    client.disconnect()
    GPIO.cleanup()