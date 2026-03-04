from flask import Flask
from flask_cors import CORS, cross_origin
import json

app = Flask(__name__)
cors = CORS(app) # allow CORS for all domains on all routes.
app.config['CORS_HEADERS'] = 'Content-Type'

@app.route('/')
def hello_world():
    return json.loads('{"x":0, "y":-1, "z":0}')

if __name__ == '__main__':
    
    app.run(host='0.0.0.0')