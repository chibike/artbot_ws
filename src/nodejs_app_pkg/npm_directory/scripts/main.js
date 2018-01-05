const electron = require('electron')
const path = require('path')
const url = require('url')

const app = electron.app
const BrowserWindow = electron.BrowserWindow

let mainWindow = null

function createWindow ()
{
  mainWindow = new BrowserWindow({ width: 1280, height: 800, titleBarStyle: 'hidden'})

  // Open the DevTools.
  mainWindow.webContents.openDevTools()

  mainWindow.loadURL(
    url.format(
    {
      pathname: path.join(__dirname, '../html/cover.html'),
      protocol: 'file:',
      slashes: true
    })
  )

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

exports.exitApp = exit_app