from flask import Flask, render_template
import flask
from waitress import serve

app = Flask(__name__)
app.debug = True


@app.route('/')
def index_view():
    return render_template('index.html')


@app.route('/timelines')
def timelines_view():
    return render_template('timelines.html')


if __name__ == '__main__':
    print('Starting robot GUI..')
    serve(app, host="0.0.0.0", port=8080)
