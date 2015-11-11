This node supports HTML UI reachable at: http://www.evee.cz/sdu/rsd

To communicate with workcell from HMI, you need to run (on a wc-connected computer):

This node:
$ roslaunch hmi_support_frobit hmi_support_frobit.launch

ROSBRIDGE:
$ roslaunch rosbridge_server rosbridge_websocket.launch


