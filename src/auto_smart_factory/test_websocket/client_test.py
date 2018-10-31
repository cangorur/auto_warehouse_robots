from websocket import create_connection
import json

ws = create_connection("ws://localhost:8887")

## SEND
print "Sending the json request"
#with open('chariot-storage.json') as data_file:
#with open('chariot-inputContainer.json') as data_file:
#with open('chariot-deliveryContainer.json') as data_file:
#with open('chariot-chargingStation.json') as data_file:
#with open('chariot-deliveryRobot.json') as data_file:
with open('chariot-conveyorBelt.json') as data_file:
    data = json.load(data_file)
print json.dumps(data)

ws.send(json.dumps(data))
print "Sent"

## RECEIVE
print "Receiving..."
result =  ws.recv()
json_data = json.loads(result)
with open('received_data.json', 'w') as outfile:
    json.dumps(json_data, outfile)

print "Received:  '%s'" % result
ws.close()
