Web Socket Interface
====================

To test the system's socket connections use the `src/auto_smart_factory/test_websocket/client_test.py`. By changing the .json files called in the python script you can receive the sensor results and able to control the actuators.

TODO: Update here with the latest version of the .json files. 

Currently:
---

## Information on Web Socket connection

port# = 8887



## JSON Files to send
### tray-driver
{
  "from": {
    "id": "device-id",
    "name": "tray-driver"
  },
  "to": {
    "id": "tray-id",
    "name": "tray"
  },
  "request": {
    "status": "getSatus"
  },
  "response": {
    "status": "tray is empty or not"
  }
}

### human-worker-driver

{
  "from": {
    "id": "device-id",
    "name": "human-worker-driver"
  },
  "to": {
    "id": "human-id",
    "name": "human-worker"
  },
  "request": {
    "task": "takeProductFromCB"
  },
  "response": {
    "availability": "yes or no"
  }
}

### conveyor belt

{
  "from":{
    "id":"device-id",
    "name":"cb-driver"
  },
  "to":{
    "id":"conveyer-belt-id",
    "name":"conveyer-belt"
  },
  "request":{
    "run":"runConveyerBelt",
    "status":"getSatus",
    "product":{
      "create":"createProduct",
      "id":"getProductId",
      "location":"getProductLocation",
    }
  },
  "response":{
   "status":"up, down, unknown, running,etc.",
   "product":{
     "id":"5436467",
     "location":"lat,long,"
   }
  }
},

{
  "from":{
    "id":"device-id",
    "name":"robot"
  },
  "to":{
    "id":"conveyer-belt-id",
    "name":"conveyer-belt"
  },
  "request":{
    "run":"runConveyerBelt",
    "status":"getSatus",
    "product":{
      "id":"getProductId",
      "location":"getProductLocation",
    }
  },
  "response":{
   "status":"up, down, unknown, running,etc."
  }
},

### transporter robot
{
    "from": {
        "id": "device-id",
        "name": "trapsorter-robot"
    },
    "to": {
        "id": "trasporter-robot-id",
        "name": "transporter-robot"
    },
    "request": {
        "run": "runRobot",
        "status": "getSatus",
        "product": {
            "putInTray": "putInTray",
            "id": "getProductId",
            "location": "getProductLocation"
        }
    },
    "response": {
        "status": "up, down, unknown, running,etc.",
        "availability": "busy or not"
    }
}
