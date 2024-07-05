# user_interface/app.py

import dash
from dash import dcc, html
from dash.dependencies import Input, Output

# Initialize the Dash app
app = dash.Dash(__name__)

# Define the layout of the app
app.layout = html.Div(children=[
    html.H1(children='Hello Dash'),

    html.Div(children='''
        Dash: A web application framework for Python.
    '''),

    dcc.Input(id='input-box', type='text'),
    html.Button('Submit', id='button'),
    html.Div(id='output-container-button',
             children='Enter a value and press submit')
])

# Define callback to update the output based on input
@app.callback(
    Output('output-container-button', 'children'),
    Input('button', 'n_clicks'),
    Input('input-box', 'value')
)
def update_output(n_clicks, value):
    if n_clicks is None:
        return 'Enter a value and press submit'
    else:
        return f'The input value is: {value}'

# Run the app
if __name__ == '__main__':
    app.run_server(debug=True)
