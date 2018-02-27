const electron = require('electron')
const path = require('path')
const url = require('url')
var SerialPort = require('serialport')

const app = electron.app
const BrowserWindow = electron.BrowserWindow

// var ros_bridge = require('./ros_middle_man').ROS_Bridge;
var audio_man = require('./audio_man').Audio_Man;

let mainWindow = null

function createWindow ()
{
  var window_properties = { width: 1024, height: 600, titleBarStyle: 'hidden', frame: true}
  mainWindow = new BrowserWindow(window_properties)
  mainWindow.setFullScreen(true)

  // Open the DevTools.
  // mainWindow.webContents.openDevTools()

  var entry_url = url.format({pathname: path.join(__dirname, '../html/cover.html'), protocol: 'file:', slashes: true })
  mainWindow.loadURL(entry_url)

  mainWindow.on('closed', function ()
  {
    // Dereference the window object, usually you would store windows
    // in an array if your app supports multi windows, this is the time
    // when you should delete the corresponding element.
    mainWindow = null
  })
}

function exit_app()
{
  if (process.platform !== 'darwin')
  {
    app.quit()
  }
}

app.on('ready', createWindow)
app.on('window-all-closed', function ()
  {
  if (process.platform !== 'darwin')
  {
    app.quit()
  }
})

app.on('activate', function ()
  {
  if (mainWindow === null)
  {
    createWindow()
  }
})

// In this file you can include the rest of your app's specific main process
// code. You can also put them in separate files and require them here.

var port = new SerialPort('/dev/serial/by-path/platform-s5p-ehci-usb-0:3.2.2:1.0-port0', {
  baudRate: 115200
});

exports.exitApp = exit_app
exports.serialPort = port;
exports.audio_man = new audio_man();
// exports.ros_bridge = ros_bridge;
