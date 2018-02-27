// This file is required by the index.html file and will
// be executed in the renderer process for that window.
// All of the Node.js APIs are available in this process.

const electron = require('electron')
const ipc = electron.ipcRenderer
const remote = electron.remote
const mainProcess = remote.require('./main')

var serial_port = mainProcess.serialPort;
const parsers = serial_port.parsers;

var external_button_pressed_event = function(data){};

serial_port.on('data', function(data)
{
    var json_data = JSON.parse(data.toString());
    external_button_pressed_event(json_data);
});