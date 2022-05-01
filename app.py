from flask import Flask, request
from models import vrp1

app = Flask(__name__)

@app.route('/')
def index():
    return '<p>Hello, world!</p>'

@app.route('/solve/<model>', methods=['GET', 'POST'])
def solve(model):
    data = request.get_json()
    if model == 'vrp1':
        return vrp1(data['locations'], data['demands'], data['capacity'])
    return {}
    # return {
    #     'routes': [[[40.008992173966995, -105.28547778746469], [40.00320668011718, -105.24633899351937]], [[40.008992173966995, -105.28547778746469], [40.00320668011718, -105.24633899351937]]]
    # }