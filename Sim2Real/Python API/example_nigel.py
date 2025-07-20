#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import autodrive

################################################################################

# Initialize vehicle(s)
nigel_1 = autodrive.Nigel()
nigel_1.id = 'V1'

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__) # '__main__'

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print('Connected!')

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    if data:

        # Vehicle perception
        nigel_1.parse_data(data, verbose=True)

        # Vehicle control
        nigel_1.cosim_mode = 0
        nigel_1.posX_command = -1.19
        nigel_1.posY_command = 0.15
        nigel_1.posZ_command = 0.0
        nigel_1.rotX_command = 0.0
        nigel_1.rotY_command = 0.0
        nigel_1.rotZ_command = 0.0
        nigel_1.rotW_command = 1.0
        
        nigel_1.throttle_command = 0 # [-1, 1]
        nigel_1.steering_command = 0 # [-1, 1]
        nigel_1.headlights_command = 2 # [0 = disabled, 1 = low beam, 2 = high beam]
        nigel_1.indicators_command = 3 # [0 = disabled, 1 = left turn indicator, 2 = right turn indicator, 3 = hazard indicator]

        ########################################################################

        json_msg = nigel_1.generate_commands(verbose=True) # Generate vehicle 1 message

        try:
            sio.emit('Bridge', data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)

################################################################################

if __name__ == '__main__':
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server
