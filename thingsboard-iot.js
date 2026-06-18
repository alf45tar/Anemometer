// CONFIGURATION
let CONFIG = {
  thingsboard_host: "eu.thingsboard.cloud", // Updated to your cloud host
  device_token: "",                         // Insert your ThingsBoard Device Token here
  sample_interval: 5000,                    // Sample every 5000ms (5 seconds)
  send_after_ticks: 3,                      // Send after 3 samples (3 * 5s = 15 seconds)
  
  // Update these IDs with your actual Shelly component IDs
  battery_component_id: "bthomesensor:200",       // ID for Battery percentage
  wind_speed_component_id: "bthomesensor:201",    // ID for Wind Speed
  temperature_component_id: "bthomesensor:202"    // ID for Temperature
};

// GLOBAL VARIABLES TO TRACK MAX VALUES AND TICKS
let max_bat = null;
let max_wind = null;
let max_temp = null;
let tick_counter = 0;

function sampleAndProcess() {
  // 1. Fetch current status from Shelly components
  let batStatus  = Shelly.getComponentStatus(CONFIG.battery_component_id);
  let windStatus = Shelly.getComponentStatus(CONFIG.wind_speed_component_id);
  let tempStatus = Shelly.getComponentStatus(CONFIG.temperature_component_id);

  // 2. Update Battery Maximum
  if (batStatus && batStatus.value !== undefined && batStatus.value !== null) {
    let currentBat = batStatus.value;
    if (max_bat === null || currentBat > max_bat) {
      max_bat = currentBat;
    }
  }

  // 3. Update Wind Speed Maximum
  if (windStatus && windStatus.value !== undefined && windStatus.value !== null) {
    let currentWind = windStatus.value;
    if (max_wind === null || currentWind > max_wind) {
      max_wind = currentWind;
    }
  }

  // 4. Update Temperature Maximum
  if (tempStatus && tempStatus.value !== undefined && tempStatus.value !== null) {
    let currentTemp = tempStatus.value;
    if (max_temp === null || currentTemp > max_temp) {
      max_temp = currentTemp;
    }
  }

  // Increment the execution counter
  tick_counter++;
  print("Sample taken (Tick " + JSON.stringify(tick_counter) + "/" + JSON.stringify(CONFIG.send_after_ticks) + "). Current max wind speed:", max_wind);

  // 5. Check if 15 seconds have passed (3 ticks)
  if (tick_counter >= CONFIG.send_after_ticks) {
    sendToThingsBoard();
  }
}

function sendToThingsBoard() {
  // Construct secure ThingsBoard HTTP Telemetry URL
  let url = "https://" + CONFIG.thingsboard_host + "/api/v1/" + CONFIG.device_token + "/telemetry";
  
  // Construct the telemetry JSON payload object
  let payload = {};
  let hasData = false;

  if (max_bat !== null) { payload.battery = max_bat; hasData = true; }
  if (max_wind !== null) { payload.wind_speed = max_wind; hasData = true; }
  if (max_temp !== null) { payload.temperature = max_temp; hasData = true; }

  if (hasData) {
    let jsonBody = JSON.stringify(payload);
    print("15s reached. Sending MAX values to ThingsBoard. Body:", jsonBody);

    // ThingsBoard requires a POST request with Content-Type application/json
    Shelly.call(
      "HTTP.Request", 
      { 
        method: "POST", 
        url: url,
        headers: {
          "Content-Type": "application/json"
        },
        body: jsonBody
      }, 
      function(response, error_code, error_message) {
        if (error_code === 0 && response.code === 200) {
          print("Data sent successfully to ThingsBoard!");
        } else {
          print("Error sending data. Code:", error_code, "HTTP Status:", response ? response.code : "N/A", "Msg:", error_message);
        }
      }
    );
  } else {
    print("No valid sensor data collected in the last 15 seconds. Skipping transmission.");
  }

  // RESET MAX VALUES AND COUNTER FOR THE NEXT 15-SECOND WINDOW
  max_bat = null;
  max_wind = null;
  max_temp = null;
  tick_counter = 0;
}

// Start the 5-second sampling loop
Timer.set(CONFIG.sample_interval, true, function() {
  sampleAndProcess();
});

// Run the first sample immediately on startup
sampleAndProcess();