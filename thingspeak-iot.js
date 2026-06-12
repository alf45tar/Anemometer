// CONFIGURATION
let CONFIG = {
  thingspeak_api_key: "",                   // Insert your ThingSpeak Write API Key here
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
    sendToThingSpeak();
  }
}

function sendToThingSpeak() {
  let url = "https://api.thingspeak.com/update?api_key=" + CONFIG.thingspeak_api_key;
  let dataParams = "";

  // Append max values to fields based on the new ordered mapping
  if (max_bat !== null)  dataParams += "&field1=" + JSON.stringify(max_bat);
  if (max_wind !== null) dataParams += "&field2=" + JSON.stringify(max_wind);
  if (max_temp !== null) dataParams += "&field3=" + JSON.stringify(max_temp);

  if (dataParams !== "") {
    let finalUrl = url + dataParams;
    print("15s reached. Sending MAX values to ThingSpeak. Params:", dataParams);

    Shelly.call("HTTP.Request", { method: "GET", url: finalUrl }, function(response, error_code, error_message) {
      if (error_code === 0 && response.code === 200) {
        print("Data sent successfully! ThingSpeak Entry ID:", response.body);
      } else {
        print("Error sending data. Code:", error_code, "Msg:", error_message);
      }
    });
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