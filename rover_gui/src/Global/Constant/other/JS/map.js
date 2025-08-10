async function initializeMap() 
{
    const isConnected = await checkConnectivity();
    if (isConnected) 
    {
      mapManager = new MapManager('cesiumContainer');
    } 
    else 
    {
      console.error("Unable to connect to the internet");
    }
}

async function checkConnectivity() 
{
    const connectionError = document.getElementById('connectionError');
    if (!connectionError)
    {
        return false;
    }

    if (!navigator.onLine)
    {
        connectionError.style.display = 'block';
        return false;
    } 
    
    try {
        await fetch('https://cesium.com/downloads/cesiumjs/releases/1.114/Build/Cesium/Cesium.js', {
            method: 'HEAD',
            mode: 'no-cors',
            cache: 'no-store'
        });
        connectionError.style.display = 'none';
        return true;
    } catch {
        connectionError.style.display = 'block';
        return false;
    }
}

document.addEventListener('DOMContentLoaded', initializeMap);

window.addEventListener('offline', function () 
{
    const connectionError = document.getElementById('connectionError');
    if (connectionError)
    {
        connectionError.style.display = 'block';
    }
});

window.addEventListener('online', async function () {
    const connection = await checkConnectivity();
    if (connection)
    {
        const connectionError = document.getElementById('connectionError');
        if (connectionError)
        {
            connectionError.style.display = 'none';
        }
        if (!mapManager)
        {
            initializeMap();
        }
        // TODO: Reinitialize mapManager after on-off-on 
    }
});