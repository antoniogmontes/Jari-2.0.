#!python3

from flask import Flask, jsonify, request
import datetime

app = Flask(__name__)

@app.route("/")
def hello():
   now = datetime.datetime.now()
   timeString = now.strftime("%Y-%m-%d %H:%M")
   return "Hello world! Hora del servidor: " + timeString

@app.route("/status", methods=['GET'])
def get_status():
    data = {
        'emotion' : 'Feliz',
        'speaker_angle' : 38.7
    }
    return jsonify(results=data) 

@app.route("/angle", methods=['POST'])
def move_angle():
    value = request.form['value']
    return value

if __name__ == "__main__":
   app.run(host='0.0.0.0', port=8080, debug=True)
