from flask import Flask, render_template, request, jsonify
from index import perform_bicep_curl
from index import perform_shoulder_press
from index import perform_push_up

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')
@app.route('/bicep_curl')
def execute_bicep_curl():
    chosen_arm = 'left'
    print('total reps are : ')
    result = perform_bicep_curl(chosen_arm)
    return render_template('result.html', result = result)
@app.route('/shoulder_press')
def shoulder_press():
    result = perform_shoulder_press()
    return render_template('result.html', result = result)
@app.route('/push_up')
def push_up():
    result = perform_push_up()
    return render_template('result.html', result = result)
if __name__ == '__main__':
    app.run(debug=True)
