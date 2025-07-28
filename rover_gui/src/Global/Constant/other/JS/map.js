async function initializeMap() 
{
    const isConnected = await checkConnectivity();
    if (isConnected) 
    {
      setupCesiumMap();
    } 
    else 
    {
      console.error("Unable to connect to the internet");
    }
}

document.addEventListener('DOMContentLoaded', initializeMap);
window.addEventListener('offline', function () 
{
    document.getElementById('connectionError').style.display = 'block';
});