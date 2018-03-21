const electron = require('electron')
const path = require('path')
const url = require('url')

const app = electron.app
const global_shortcut = electron.globalShortcut
const BrowserWindow = electron.BrowserWindow

// var ros_bridge = require('./ros_middle_man').ROS_Bridge;
var a = require('./audio_man').Audio_Man
var b = require('./button_man').Button_Man

const audio_man = new a()
const button_man = new b()

let mainWindow = null

function exit_app()
{
  if (process.platform !== 'darwin')
  {
    app.quit()
  }
}

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
    mainWindow = null
  })

  global_shortcut.register('CommandOrControl+X', exit_app);

  exports.exitApp = exit_app;
  exports.audio_man = audio_man;
  exports.button_man = button_man;
  // exports.ros_bridge = ros_bridge;
}

app.on('ready', createWindow)
app.on('window-all-closed', function ()
  {
  if (process.platform !== 'darwin')
  {
    audio_man.stop_all();
    app.quit();
  }
})

app.on('activate', function ()
  {
  if (mainWindow === null)
  {
    createWindow();
  }
})
