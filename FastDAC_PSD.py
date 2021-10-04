from scipy.fftpack import fft, fftfreq
from scipy import signal
import plotly
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.express as px
import time
import serial
import numpy as np
import struct
import FastDAC as FD
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import time
import dash_table
import pandas as pd
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning) 

fd = FD.FastDAC('COM5', baudrate=1750000, timeout=1, verbose=False)            # Optical cable
#fd = FD.FastDAC('COM6', baudrate=57600, timeout=1, verbose=False)             # USB
FastDAC_ID = fd.IDN()

def PSD(duration, channels=[0, ]):
    ts = time.time()
    xper = []
    yper = []
        
    c_time = list()
    for c in channels:
        t_read = int(fd.READ_CONVERT_TIME(channel=c))
        if t_read not in c_time:
            c_time.append(t_read)

    #assert len(c_time) == 1

    c_freq = 1/(c_time[0]*10**-6)  # Hz
    measure_freq = c_freq/len(channels)
    num_bytes = int(np.round(measure_freq*duration))

    cmd = "SPEC_ANA,{},{}\r".format("".join(str(ac) for ac in channels), num_bytes)

    if not fd.ser.is_open:
        fd.ser.open()

    fd.ser.write(bytes(cmd, "ascii"))
    channel_readings = {ac: list() for ac in channels}
    voltage_readings = []

    try:
        while fd.ser.in_waiting > 24 or len(voltage_readings) <= num_bytes/2:
            buffer = fd.ser.read(24)
            info = [buffer[i:i+2] for i in range(0, len(buffer), 2)]
            for two_b in info:
                int_val = fd.two_bytes_to_int(two_b)
                voltage_reading = fd.map_int16_to_mV(int_val)
                voltage_readings.append(voltage_reading)

    except:
        fd.ser.close()
        raise

    fd.ser.close()

    for k in range(0, len(channels)):
        channel_readings[k] = voltage_readings[k::len(channels)]
        channel_readings[k] = np.array(channel_readings[k])

        f, Pxx_den = signal.periodogram(channel_readings[k], measure_freq)
        xper.append([f])
        yper.append([Pxx_den])

    return xper, yper, str(num_bytes), str(time.time() - ts)

X = [[],[],[],[]]
Y = [[],[],[],[]]

df = pd.DataFrame()
data  = {'col1':'-', 'col2':'-', 'col3':'-', 'col4':'-'}
df = df.append(data, ignore_index=True)

app = dash.Dash(__name__)
app.layout = html.Div([

    html.Div([dcc.Graph(id="live-graph", animate=True)], style={'width': '150%', 'height':'150%'}),
    dcc.Interval(id="graph-update", interval=2000, n_intervals=0),
    html.Div([
        dcc.Dropdown(
            id='avg-dropdown', 
            options=[
                {'label': 'Averaging', 'value': 'AVG'},
                {'label': 'Normal', 'value': 'NRML'}],
            value='AVG')
            ], 
            style={'width': '15%', "margin-left": "30px"}),
        
    html.Div([
        dash_table.DataTable(
            id='table', 
            columns=[{'name': 'FastDAC ID', 'id': 'col1'},
                    {'name': 'Bytes/channel', 'id': 'col2'},
                    {'name': 'Run time', 'id': 'col3'},
                    {'name': 'Channels', 'id': 'col4'}],
            data=df.to_dict('records'),
            #fill_width=False,
            style_cell={'textAlign': 'left', "margin-left": "30px"},
            style_header={
                'backgroundColor': 'rgb(230, 230, 230)',
                'fontWeight': 'bold'})
                ], 
            style={'width': '40%', 'display': 'inline-block', "margin-left": "30px"})
    ])

@app.callback(
    Output(component_id='avg-dropdown', component_property='value'),
    [Input(component_id='avg-dropdown', component_property='options')])

@app.callback(
    Output(component_id='live-graph', component_property='figure'),
    Output(component_id='table', component_property='data'),
    [Input(component_id='graph-update', component_property='n_intervals'),
    Input(component_id='avg-dropdown', component_property='value')])

def update_graph(input_data, selected_avg):
    channel_arr = [1,2]
    psd = PSD(1, channels=channel_arr)
    fig = make_subplots(rows=[1,2,2,2][len(channel_arr)-1], cols=[1,1,2,2][len(channel_arr)-1])
    fig.update_layout(height=600, width=1000, title_text="FastDAC PSD", legend_title="Channels")
    fig.update_yaxes(type='log', title_text='Potential [mV]')
    fig.update_xaxes(title_text='Frequency [Hz]')

    for k in range(0, len(channel_arr)):
        X[k].append(psd[0][k][0])
        Y[k].append(psd[1][k][0])
        
        if len(X[k])<5 and selected_avg=='AVG':
            xnew=np.mean(X[k], axis=0)
            ynew=np.mean(Y[k], axis=0)
            
        elif selected_avg=='AVG':
            xnew=np.mean(X[k][-6:-1], axis=0)
            ynew=np.mean(Y[k][-6:-1], axis=0)
         
        elif selected_avg=='NRML':
            xnew=psd[0][k][0]
            ynew=psd[1][k][0]
                
        fig.add_trace(
            go.Scatter(
                x=xnew, 
                y=ynew, 
                name=str(channel_arr[k])), 
                row=[1,2,1,2][k], 
                col=[1,1,2,2][k])

    df = pd.DataFrame()
    data={'col1':FastDAC_ID, 'col2':psd[2], 'col3':str(round(float(psd[3]), 2)), 'col4':str(channel_arr)}
    df = df.append(data, ignore_index=True)
    return fig, df.to_dict('records')

if __name__ == '__main__':
    app.run_server(debug=True)
