from crypt import methods
from flask import Flask

app = Flask(__name__)

@app.route('/')
def index():
    return '<p>Hello, world!</p>'

@app.route('/solve/<model>', methods=['GET', 'POST'])
def solve(model):
    print(model)
    return model