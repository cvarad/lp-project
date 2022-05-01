from flask import Flask, request
from flask_cors import CORS
from models import vrp1, vrp3, vrp4

app = Flask(__name__)
CORS(app)
solvers = {'vrp1': vrp1, 'vrp3': vrp3, 'vrp4': vrp4}

@app.route('/')
def index():
    return '<p>Hello, world!</p>'

@app.route('/solve/<model>', methods=['GET', 'POST'])
def solve(model):
    if model not in solvers:
        return {'status': 'unknown model'}

    data = request.get_json()
    return solvers[model](data['locations'], data['demands'], data['capacity'])

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
