
import time
import serial
import numpy as np
from scipy import signal
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.express as px
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output, State
import dash_table
import pandas as pd
import warnings
#warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

def PSD(port, duration, channels=[0, ]):

    s = serial.Serial(port, 1750000, timeout=1)
    #s.reset_input_buffer()
    #s.reset_output_buffer()
    #s.read_all()

    def Query(command):

        if not s.is_open:
            s.open()
        s.write(command)
        data = s.readline()
        data = data.decode('ascii', errors='ignore').rstrip('\r\n')
        s.close()
        return data

    FastDAC_ID = Query(b"*IDN?\r")

    ts = time.time()
    xper = []
    yper = []

    convert_time = list()
    for c in channels:
        cmd = bytes("READ_CONVERT_TIME,{}\r".format(c), 'ascii')
        read_time_bytes = Query(cmd)
        read_time = int(read_time_bytes)

        if read_time not in convert_time:
            convert_time.append(read_time)

    assert len(convert_time) == 1

    c_freq = 1/(convert_time[0]*10**-6)  # Hz
    measure_freq = c_freq/len(channels)
    num_bytes = int(np.round(measure_freq*duration))

    cmd = "SPEC_ANA,{},{}\r".format("".join(str(ch) for ch in channels), num_bytes)

    if not s.is_open:
        s.open()
    s.write(bytes(cmd,'ascii'))

    channel_readings = {ac: list() for ac in channels}
    voltage_readings = []
    try:
        while s.in_waiting > 24 or len(voltage_readings) <= num_bytes/2:
            buffer = s.read(24)
            info = [buffer[i:i+2] for i in range(0, len(buffer), 2)]
            for two_bytes in info:
                int_val = int.from_bytes(two_bytes, 'big')
                voltage_reading = (int_val - 0) * (20000.0) / (65536.0) - 10000.0
                voltage_readings.append(voltage_reading)
    except:
        s.close()
        raise

    s.close()

    for k in range(0, len(channels)):
        channel_readings[k] = voltage_readings[k::len(channels)]
        channel_readings[k] = np.array(channel_readings[k])

        f, Pxx_den = signal.periodogram(channel_readings[k], measure_freq)
        xper.append([f])
        yper.append([Pxx_den])

    return xper, yper, str(num_bytes), str(time.time() - ts), FastDAC_ID


X = [[],[],[],[]]
Y = [[],[],[],[]]
PORT = [0,'COM5']
DUR = [0,1]

df = pd.DataFrame()
data  = {'col1':'-', 'col2':'-', 'col3':'-', 'col4':'-'}
df = df.append(data, ignore_index=True)

app = dash.Dash(__name__)
app.layout = html.Div([
    
    html.Div([

        html.Div([dcc.Graph(id="live-graph", animate=True)], style={'width': '50%', 'height':'100%'}),
        dcc.Interval(id="graph-update", interval=2000, n_intervals=0) ]),

    html.Div([

        dash_table.DataTable(
            id='table', 
            columns=[{'name': 'FastDAC ID', 'id': 'col1'},
                    {'name': 'Bytes/channel', 'id': 'col2'},
                    {'name': 'Run time', 'id': 'col3'},
                    {'name': 'Channels', 'id': 'col4'}],
            data=df.to_dict('records'),
            style_cell={'textAlign': 'left', "margin-left": "30px"},
            style_table = {'width':'40%', "margin-bottom":"30px"},
            style_header={
                'backgroundColor': 'rgb(230, 230, 230)',
                'fontWeight': 'bold'}),

        html.Div([ 
        html.Label(['Port: '], style={'font-weight': 'bold', 'margin-left': '15px',"text-align": "right","offset":0}),
        dcc.Input(id='enter-port', type='text', value='COM5', style={'width': '40%', 'height':'50%'}),
        html.Button('OK', id='enter-port-button', n_clicks=0, style={"margin-bottom": "10px"}),
        ], style = {'display': 'inline-block'}),

        html.Div([ 
        html.Label('FastDAC ID = ', style={"margin-bottom": "10px", "text-align": "right","offset":0}),
        html.Label(children=str(df.to_dict('records')[0]['col1']), id = 'label'),
        ], style = {'display': 'inline-block'}),


        html.Div([
        html.Label(children=['Duration (s): '], style={'font-weight': 'bold', "text-align": "right","offset":0, 'margin-left': '15px'}),
        dcc.Input(id='enter-duration', type = 'text',value='1', style={'width': '9%', "margin-bottom": "10px"}),
        html.Button('OK', id='enter-duration-button', n_clicks=0),
        html.Label('Runtime (s) = ', style={"margin-bottom": "10px", "text-align": "right","offset":0, 'margin-left': '65px'}),
        html.Label(children=str(df.to_dict('records')[0]['col3']), id = 'label2'),
        ]),

        html.Div([
        html.Label(['Show channels: '], style={'font-weight': 'bold', "text-align": "right","offset":0, 'display': 'inline-block', 'margin-left': '15px'}),
        dcc.Checklist(id='channels-checklist', 
                options=[
                    {'label': '0', 'value': 0},
                    {'label': '1', 'value': 1},
                    {'label': '2', 'value': 2},
                    {'label': '3', 'value': 3}], 
                value=[0],
                style={"margin-bottom": "10px", 'display': 'inline-block', 'margin-left': '15px'}),
        ]),

        html.Div([
        dcc.Dropdown(
                id='avg-dropdown', 
                options=[
                    {'label': 'Average over 1 cycle:', 'value': 1},
                    {'label': 'Average over 2 cycles:', 'value': 2},
                    {'label': 'Average over 3 cycles:', 'value': 3},
                    {'label': 'Average over 4 cycles:', 'value': 4},
                    {'label': 'Average over 5 cycles:', 'value': 5},
                    {'label': 'Average over 6 cycles:', 'value': 6}],
                value=5,
                style={'width':'75%', "margin-bottom": "10px", 'margin-left': '10px'}),
        ]),

        html.Div([
        dcc.Dropdown(
                id='axes-dropdown', 
                options=[
                    {'label': 'Log Axis', 'value': 'log'},
                    {'label': 'Linear Axis', 'value': 'linear'}], 
                value='log', 
                style={'width':'75%', "margin-bottom": "10px", 'margin-left': '10px'}),
        ]),

        ], style = {'display': 'inline-block', 'margin-left': '15px', 'border': '2px green solid'})
    ], style={'width':'150%'})

@app.callback(
    Output(component_id='live-graph', component_property='figure'),
    Output(component_id='table', component_property='data'),
    Output(component_id = 'label', component_property='children'),
    Output(component_id = 'label2', component_property='children'),
    [Input(component_id='graph-update', component_property='n_intervals'),
    Input(component_id='avg-dropdown', component_property='value'),
    Input(component_id='axes-dropdown', component_property='value'),
    Input(component_id='channels-checklist', component_property='value'),
    Input(component_id='enter-port-button', component_property='n_clicks'),
    Input(component_id='enter-duration-button', component_property='n_clicks'),
    State(component_id='enter-port', component_property='value'),
    State(component_id='enter-duration', component_property='value')
    ])


def update_graph(input_data, selected_avg, selected_axes, channel_arr, n_clicks1, n_clicks2, port, dur):

    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]

    if 'enter-port-button' in changed_id:
        PORT.append(port)
        
    if 'enter-dur-button' in changed_id:
        DUR.append(dur)
    
    psd = PSD(str(PORT[-1]), float(DUR[-1]), channel_arr)

    fig = make_subplots(rows=[1,2,2,2][len(channel_arr)-1], cols=[1,1,2,2][len(channel_arr)-1])
    fig.update_layout(height=600, width=1000, title_text="FastDAC PSD", legend_title = "channels")
    fig.update_yaxes(type=selected_axes, title_text='Potential [mV]')
    fig.update_xaxes(title_text='Frequency [Hz]')
    fig.update_layout(showlegend=False)

    for k in range(0, len(channel_arr)):
        X[k].append(psd[0][k][0])
        Y[k].append(psd[1][k][0])
        
        if len(X[k])<selected_avg:
            xnew=np.mean(X[k][-len(X[k]):-1], axis=0)
            ynew=np.mean(Y[k][-len(X[k]):-1], axis=0)

        elif selected_avg==1:
            xnew=psd[0][k][0]
            ynew=psd[1][k][0]
            
        else:
            xnew=np.mean(X[k][-selected_avg:-1], axis=0)
            ynew=np.mean(Y[k][-selected_avg:-1], axis=0)

        fig.add_trace(
            go.Scatter(
                x=xnew[15:], 
                y=ynew[15:], 
                name=str(channel_arr[k])), 
                row=[1,2,1,2][k],
                col=[1,1,2,2][k])

    #fig.update_layout(showlegend=True)

    
    df = pd.DataFrame()
    data={'col1':psd[4], 'col2':psd[2], 'col3':str(round(float(psd[3]), 2)), 'col4':str(channel_arr)}
    df = df.append(data, ignore_index=True)
    return fig, df.to_dict('records'), psd[4], str(round(float(psd[3]), 2))

if __name__ == '__main__':
    app.run_server(debug=True)



