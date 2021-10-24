# find port name by typing "ls /dev/tty.usb*" in terminal

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
from dash_extensions import Download
#from dash_extensions.snippets import send_file
import pandas as pd
#import warnings
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

    #assert len(convert_time) == 1

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
PORT = [0,'COM4']
DUR = [0,1]

app = dash.Dash(__name__, title='FastDAC Spectrum Analyzer', update_title='Loading...')
#app.scripts.config.serve_locally = True

app.layout = html.Div([

    html.Div([

        html.Div([
            dcc.Graph(id="live-graph", animate=True)
            ], style={'width': '75%', 
            'height':'100%', 
            'margin-left': '15px', 
            'margin-top': '15px', 
            'margin-bottom': '15px', 
            'border': '3px black solid'}
            ),

            dcc.Interval(id="graph-update", interval=1500, n_intervals=0)]),

    html.Div([

        html.Div([ 
        html.Label(['Port:'], style={'font-weight': 'bold', 'margin-left': '15px','margin-right':'56px'}),
        dcc.Input(id='enter-port', type='text', value=str(PORT[-1]), style={'width': '40%', 'height':'50%'}),
        html.Button('OK', id='enter-port-button', n_clicks=0, style={"margin-bottom": "10px"}),
        ], style={'margin-top':'15px'}),

        html.Div([
        html.Label(children=['Duration (s): '], style={'font-weight': 'bold', "text-align": "right","offset":0, 'margin-left': '15px'}),
        dcc.Input(id='enter-duration', type = 'text',value=str(DUR[-1]), style={'width': '40%', "margin-bottom": "10px"}),
        html.Button('OK', id='enter-duration-button', n_clicks=0),
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
                style={"margin-bottom": "10px", 'display': 'inline-block','margin-left': '15px','margin-right': '10px'}),
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
                style={ "margin-bottom": "15px", 'width':'90%','margin-left': '10px','margin-right': '10px'}),
        ]),

        html.Div([
            dcc.Dropdown(
                id='axes-dropdown', 
                options=[
                    {'label': 'Log Axis', 'value': 'log'},
                    {'label': 'Linear Axis', 'value': 'linear'}], 
                value='log', 
                style={ "margin-bottom": "15px", 'margin-left': '10px','margin-right': '10px',  'width':'90%'}),
        ]),

        ], style = {'width':'25%',
            'height':'25%',
            'border': '3px black solid', 
            'backgroundColor':'white',
            'margin-left':'15px', 
            'display': 'inline-block'}),

        html.Div([
            
            html.Div([html.Label('FastDAC ID: ', style={'font-weight': 'bold', 'margin-right':'10px'}),
                    html.Label(children='-', id = 'label1')], 
                    style={"margin-top": "15px", 
                    "margin-left": "15px", 
                    "margin-right": "15px"}),

            html.Div([html.Label('Runtime (s): ', style={'font-weight': 'bold', 'margin-right':'15px'}),
                html.Label(children='-', id = 'label2')], 
                style={
                    "margin-top": "15px",  
                    "margin-bottom": "15px", 
                    "margin-left": "15px", 
                    "margin-right": "15px"}),

            html.Div([html.Label('bytes / s: ', style={'font-weight': 'bold', 'margin-right':'40px'}),
                html.Label(children='-', id = 'label3')], 
                style={
                    "margin-top": "15px",  
                    "margin-bottom": "15px", 
                    "margin-left": "15px", 
                    "margin-right": "15px"}),

            ], style={'vertical-align':'top', 
                'border': '3px black solid', 
                'backgroundColor':'white',
                'display': 'inline-block', 
                'margin-left':'15px', 
                'height':'35%', 
                'width':'48.5%'})

    ], style={'backgroundColor':'#BFBAB9', 
        'padding-top':'1%', 
        'padding-bottom':'50%',
        'padding-right':'0%',
        'padding-left':'0%'})

@app.callback(
    Output(component_id='live-graph', component_property='figure'),
    Output(component_id = 'label1', component_property='children'),
    Output(component_id = 'label2', component_property='children'),
    Output(component_id = 'label3', component_property='children'),
    [Input(component_id='graph-update', component_property='n_intervals'),
    Input(component_id='avg-dropdown', component_property='value'),
    Input(component_id='axes-dropdown', component_property='value'),
    Input(component_id='channels-checklist', component_property='value'),
    Input(component_id='enter-port-button', component_property='n_clicks'),
    Input(component_id='enter-duration-button', component_property='n_clicks'),
    State(component_id='enter-port', component_property='value'),
    State(component_id='enter-duration', component_property='value')]
    )

def update_graph(input_data, selected_avg, selected_axes, channel_arr, n_clicks1, n_clicks2, port, dur):

    changed_id = [p['prop_id'] for p in dash.callback_context.triggered][0]

    if 'enter-port-button' in changed_id:
        PORT.append(port)
        
    if 'enter-dur-button' in changed_id:
        DUR.append(dur)

    #print(dur)
    #print(DUR[-1])
    
    psd = PSD(str(PORT[-1]), float(DUR[-1]), channel_arr)

    fig = make_subplots(rows=[1,2,2,2][len(channel_arr)-1], cols=[1,1,2,2][len(channel_arr)-1])
    fig.update_layout(title_text="FastDAC Spectrum Analyzer", title_x=0.5, legend_title = "channels")
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

    #if 'btn_image' in changed_id:
    #    dcc.send_file("./DASH.png")

    return fig, psd[4], psd[3], psd[2]

if __name__ == '__main__':
     app.run_server(host= '0.0.0.0', debug=False)
