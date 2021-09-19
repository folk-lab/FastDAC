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

fd = FD.FastDAC('COM13', baudrate=1750000, timeout=1, verbose=False)            # Optical cable
#fd = FD.FastDAC('COM6', baudrate=57600, timeout=1, verbose=False)              # USB
FastDAC_ID = fd.IDN()

def PSD(duration, channels=[0, ]):
    ts = time.time()
    xfft = []
    yfft = []
    xper = []
    yper = []
    xwel = []
    ywel = []
        
    c_time = list()
    for c in channels:
        t_read = int(fd.READ_CONVERT_TIME(channel=c))
        if t_read not in c_time:
            c_time.append(t_read)

    assert len(c_time) == 1

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
        while fd.ser.in_waiting > 24+15 or len(voltage_readings) <= num_bytes/2:
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

        y = fft(channel_readings[k])
        yfft.append( [ 1/num_bytes * np.abs(y[0:num_bytes//2]) ] )
        T = 1/measure_freq
        xfft.append( [ fftfreq(num_bytes, T)[:num_bytes//2] ] )

        f, Pxx_den = signal.periodogram(channel_readings[k], measure_freq)
        xper.append([f])
        yper.append([Pxx_den])

        f, Pxx_den = signal.welch(channel_readings[k], measure_freq) #, nperseg=24)
        xwel.append([f])
        ywel.append([Pxx_den])

    return xfft, yfft, xper, yper, xwel, ywel, str(num_bytes), str(time.time() - ts)

X = []
Y = []

app = dash.Dash(__name__)
app.layout = html.Div([
    dcc.Graph(id="live-graph", animate=True),
    dcc.Interval(id="graph-update", interval=1500, n_intervals=0),
    dcc.Dropdown(
        id='avg-dropdown',
        options=[
            {'label': 'Averaging', 'value': 'AVG'},
            {'label': 'Normal', 'value': 'NRML'}
        ],
        value='AVG'
    ),

    html.Hr(),
    dcc.Dropdown(
        id='psd-dropdown',
        options=[
            {'label': 'Fast Fourier Transform (FFT)', 'value': 'FFT'},
            {'label': 'Periodogram', 'value': 'PER'},
            {'label': 'Welch', 'value': 'WEL'}
        ],
        value='FFT'
    ),

    html.Hr()
])

@app.callback(
    Output('avg-dropdown', 'value'),
    [Input('avg-dropdown', 'options')])

@app.callback(
    Output('psd-dropdown', 'value'),
    [Input('psd-dropdown', 'options')])

@app.callback(
    Output('live-graph', 'figure'),
    [Input('graph-update', 'n_intervals'),
    Input('psd-dropdown', 'value'),
    Input('avg-dropdown', 'value')])

def update_graph(input_data, selected_psd, selected_avg):
    channel_arr = [0,]
    psd = PSD(1, channels=channel_arr)
    fig = make_subplots(rows=[1,2,2,2][len(channel_arr)-1], cols=[1,1,2,2][len(channel_arr)-1])
    fig.update_layout(height=600, width=1000, title_text="PSD", legend_title="Legend")
    fig.update_yaxes(type='log', title_text='Potential [mV]')
    fig.update_xaxes(title_text='Frequency [Hz]')
    fig.add_annotation(text="num_bytes="+psd[6]+", Runtime="+psd[7]+'s'+", FastDAC ID="+FastDAC_ID+", channels="+str(channel_arr),
                                xref="paper", yref="paper",x=0, y=1.05, showarrow=False)

    if selected_psd=='FFT':
        x_i = 0
        y_i = 1
    elif selected_psd=='PER':
        x_i = 2
        y_i = 3
    elif selected_psd=='WEL':
        x_i = 4
        y_i = 5

    for k in range(0, len(channel_arr)):
        X.append(psd[x_i][k][0])
        Y.append(psd[y_i][k][0])
        
        if len(X)<5 and selected_avg=='AVG':
            xnew=np.mean(X, axis=0) 
            ynew=np.mean(Y, axis=0)
            

        elif selected_avg=='AVG':
            xnew=np.mean(X[-6:-1], axis=0)
            ynew=np.mean(Y[-6:-1], axis=0)
         
        elif selected_avg=='NRML':
            xnew=psd[x_i][k][0]
            ynew=psd[y_i][k][0]
                
        fig.add_trace(
            go.Scatter(
                x=xnew, 
                y=ynew, 
                name='Ch.'+ str(channel_arr[k])), 
                row=[1,2,1,2][k], 
                col=[1,1,2,2][k]
                )

    return fig


if __name__ == '__main__':
    app.run_server(debug=True)

