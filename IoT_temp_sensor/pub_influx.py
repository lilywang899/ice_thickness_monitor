from paho.mqtt import client as mqtt_client
import time
import random
import json

broker = "localhost"
port = 1883
topic_temp = "python/mqtt/temp"
topic_humi = "python/mqtt/humi"
CLIENT_ID = f'publish-{random.randint(0, 1000)}'

def on_connect(client,userdata,flags,rc):
    if rc == 0:
        print("connected to MQTT broker!")
    else:
        print("failed to connect, return code %d\n", rc)

FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60

def on_disconnect(client, userdata, rc):
    print("disconnected with result conde: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while reconnect_count < MAX_RECONNECT_COUNT:
        ("reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            print("reconnected successfully")
            return
        except Exception as err:
            print("%s. Reconnect failed. Retrying...", err)
        
        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    print("reconnect failed after %s attempts. Exiting...", reconnect_count)

def connect_mqtt():
    client = mqtt_client.Client(CLIENT_ID)
    client.on_connect = on_connect #assigns callback function to client
    client.connect(broker, port)
    client.on_disconnect = on_disconnect
    return client

# with open('data.json','r') as file:
#     data = json.load(file)
#     data_list = data["data"]

msg_id = {'temp': None, 'humi': None}

def publish(client):
    msg_count = 1
    global msg_id
    while True:
        time.sleep(1)
        # temp = f"messages: {data_list[msg_count]["temp"]}"
        # humi = f"messages: {data_list[msg_count]["humidity"]}"
        # temp = data_list[msg_count]["temp"]
        # humi = data_list[msg_count]["humidity"]
        temp = msg_count*10
        print(temp)
        humi = msg_count*11
        print(humi)
        result= client.publish(topic_temp,temp) #result is a list[2]
        msg_id['temp'] = result[1]
        result= client.publish(topic_humi,humi)
        msg_id['humi'] = result[1]        
        msg_count += 1
        

def on_publish(client,userdata,mid):
    if msg_id['temp'] == mid:
        print(f"send msg id'{mid}' to topic '{topic_temp}'")
    elif msg_id['humi'] == mid:
        print(f"send msg id'{mid}' to topic '{topic_humi}'")
    else:
        print(f"failed to send message to topic")

    return

def run():
    client = connect_mqtt()
    client.loop_start() 
    client.on_publish = on_publish
    publish(client)
    client.loop_stop()

if __name__ == '__main__':
    run()