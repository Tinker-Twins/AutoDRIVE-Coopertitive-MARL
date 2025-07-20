#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
import autodrive

################################################################################

# Initialize vehicle(s)
f1tenth_1 = autodrive.F1TENTH()
f1tenth_1.id = 'V1'

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
        f1tenth_1.parse_data(data, verbose=True)

        # Vehicle control
        f1tenth_1.cosim_mode = 1
        f1tenth_1.posX_command = 14.8
        f1tenth_1.posY_command = -6.84
        f1tenth_1.posZ_command = 0.0
        f1tenth_1.rotX_command = 0.0
        f1tenth_1.rotY_command = 0.0
        f1tenth_1.rotZ_command = -0.7071068
        f1tenth_1.rotW_command = 0.7071068
        
        f1tenth_1.throttle_command = 0 # [-1, 1]
        f1tenth_1.steering_command = 0 # [-1, 1]

        ########################################################################

        json_msg = f1tenth_1.generate_commands(verbose=True) # Generate vehicle 1 message

        try:
            sio.emit('Bridge', data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)

################################################################################

if __name__ == '__main__':
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server
