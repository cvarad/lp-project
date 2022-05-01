from flask import Flask, request
from flask_cors import CORS
from models import vrp1

app = Flask(__name__)
CORS(app)

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

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
