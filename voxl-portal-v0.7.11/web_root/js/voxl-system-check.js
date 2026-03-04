// variable for passing the health check
var check = true;
// function that writes a row to the table for a test and its result
function healthCheckWriteTestRow(test,result,passed) {
	var tableBody = document.getElementById('health-check-table').getElementsByTagName('tbody')[0];
    var row = document.createElement('tr');
    var testCell = document.createElement('td');
    testCell.innerText = test;
    row.appendChild(testCell);
	const resultCell = document.createElement('td');
	if (passed == 'true' || passed == 'True' || passed == true){
		resultCell.innerText = result;
    	resultCell.style.color = 'green';
	}
	else {
		check = false;
		resultCell.innerText = result;
    	resultCell.style.color = 'red';
	}
    row.appendChild(resultCell);
    tableBody.appendChild(row);
}

// function that writes a row to the top of the table (used for adding the result at the end)
function healthCheckWriteTopRow(checkResult) {
    
    var tableBody = document.getElementById('health-check-table').getElementsByTagName('tbody')[0];
    var headerRow = document.createElement('tr');
    var headerCell = document.createElement('td');
    headerCell.colSpan = 2; 
	headerCell.style.textAlign = 'center';
	headerCell.style.fontSize = 'larger';
	if (checkResult == false) {
		headerCell.innerHTML = '<b>' + 'Health Check Failed' + '</b>'; 
		headerCell.style.color = 'red';
	}
	else {
		headerCell.innerHTML = '<b>' + 'Health Check Passed' + '</b>'; 
		headerCell.style.color = 'green';
	}
    headerRow.appendChild(headerCell);
    tableBody.insertBefore(headerRow, tableBody.firstChild);
}

// function that writes a header row (used for each of the titles of the services)
function tableWriteHeaderRow(tableID,headerText,textColor) {
    
    var tableBody = document.getElementById(tableID).getElementsByTagName('tbody')[0];
    var headerRow = document.createElement('tr');
    var headerCell = document.createElement('td');
	if (tableID == 'health-check-table') {
		headerCell.colSpan = 2
	}
	else {
		headerCell.colSpan = 4; 
	} 
    headerCell.innerHTML = '<b>' + headerText + '</b>'; 
	headerCell.style.textAlign = 'center';
	headerCell.style.color = textColor;
    headerRow.appendChild(headerCell);
    tableBody.appendChild(headerRow);
}
function healthCheckWriteTopTest(){
	var tableBody = document.getElementById('health-check-table').getElementsByTagName('tbody')[0];
    var row = document.createElement('tr');
    var testCell = document.createElement('td');
    testCell.innerHTML = '<b>Test</b>';
	testCell.style.color = 'blue';
    row.appendChild(testCell);
	var resultCell = document.createElement('td');
	resultCell.innerHTML = '<b>Result</b>'
	resultCell.style.color = 'blue'
	row.appendChild(resultCell);
    tableBody.appendChild(row);
}

// works through each service and calls the print functions for the tests
healthCheckPrintResults = function(healthData){


	healthCheckWriteTopTest();
	// Display the SDK:$(document).ready(function() { 

	// first section is the image sensors:

	var voxlImageSensors = healthData['compute'];
	if (voxlImageSensors) {
		tableWriteHeaderRow('health-check-table',"---------------- VOXL Image Sensors ----------------")
	
		// extract all image sensors into an array
		var imageSensors = Object.keys(voxlImageSensors);

		//total number of image sensors

		var NumImageSensors = imageSensors.length;

		// loop through and for each image sensor display its result

		for (var i =0; i < NumImageSensors; i++) {
			var imageSens = imageSensors[i];
			var result = voxlImageSensors[imageSens]['probe'];
			var passed = voxlImageSensors[imageSens]['result'];
			healthCheckWriteTestRow(imageSens,result,passed);
		}
	}


	//second section is the camera server
	var voxlCameraServer = healthData['voxl-camera-server']
	if (voxlCameraServer) {
		tableWriteHeaderRow('health-check-table',"---------------- VOXL-Camera-Server ----------------")

		var cams = Object.keys(voxlCameraServer);
		var numCams = cams.length;

		for (var i =0; i < numCams; i++) {
			var cam = cams[i];
			var result = voxlCameraServer[cam];
			if (result == 'True' || result == 'true' || result == 'false'|| result == 'False' || result == 'error' || result == false || result == true ) {
				passed = result;
			}
			else {
				passed = 'True';
			}
			healthCheckWriteTestRow(cam,result,passed);
		}
	}

	// third section is the mavlink server 
	var voxlMavlinkServer = healthData['voxl-mavlink-server']
	if (voxlMavlinkServer) {
		tableWriteHeaderRow('health-check-table',"---------------- VOXL-MavLink-Sever ----------------")
		
		var mavLinks = Object.keys(voxlMavlinkServer);

		var numMavLinks = mavLinks.length;

		for (var i =0; i < numMavLinks; i++) {
			var mavLink
			// PAULS VERSION : = mavLinks[i];
			var result = voxlMavlinkServer[mavLink];
			if (result == 'True' || result == 'true' || result == 'false'|| result == 'False' || result == 'error' || result == false || result == true ) {
				passed = result;
			}
			else {
				passed = 'True';
			}
			healthCheckWriteTestRow(mavLink,result,passed);
		}

	}

	// fourth section is the voxl-px4
	var voxlPx4 = healthData['voxl-px4'];
	if (voxlPx4) {
		tableWriteHeaderRow('health-check-table',"---------------- VOXL-Px4 ----------------")
		
		var px4Data = Object.keys(voxlPx4);

		var numPx4Data = px4Data.length;

		// i < 4 is pre-sensor data
		for (var i =0; i < numPx4Data; i++) {
			var px4 = px4Data[i];
			var result = voxlPx4[px4];

			if (i < 4) {
				if (px4 == 'error' || px4 == 'Error'){
					if (result == 'true' || result == 'True'){
						passed = 'false';
					}
					else {
						passed = 'true'; 
					}
				}
				else {
					if (result == 'True' || result == 'true' || result == 'false'|| result == 'False' || result == 'error' || result == false || result == true ) {
						passed = result;
					}
					else {
						passed = 'True';
					}
				}
				healthCheckWriteTestRow(px4,result,passed);
			}
			else {
				var sensorData = result;
				var sensorKeys = Object.keys(sensorData);
				var sensorLength = sensorKeys.length;
				var passed = sensorData['result'];
				tableWriteHeaderRow('health-check-table',px4 + ': ','blue');
				for (var j = 0; j < sensorLength; j++){
					var sensorKey = sensorKeys[j]
					healthCheckWriteTestRow(sensorKey,sensorData[sensorKey],passed);
					
				}
				
			}
		}
	}

			// fifth section is the voxl imu server
	var voxlImuServer = healthData['voxl-imu-server']
	if (voxlImuServer) {
		tableWriteHeaderRow('health-check-table',"---------------- VOXL-IMU-Server ----------------")
		
		var imuProperties = Object.keys(voxlImuServer);

		var numProps = imuProperties.length;

		for (var i =0; i < numProps; i++) {
			var prop = imuProperties[i];
			var result = voxlImuServer[prop];
			if (result == 'True' || result == 'true' || result == 'false'|| result == 'False' || result == 'error' || result == false || result == true ) {
				passed = result;
			}
			else {
				passed = 'True';
			}
			healthCheckWriteTestRow(prop,result,passed);
		}
	}

	 // sixth section is the QVIO server
	var voxlQvioServer = healthData['voxl-qvio-server']
	if (voxlQvioServer) {
		tableWriteHeaderRow('health-check-table',"---------------- VOXL-QVIO-Server ----------------")
		
		var QvioProperties = Object.keys(voxlQvioServer);

		var numProps = QvioProperties.length;

		for (var i =0; i < numProps; i++) {
			var prop = QvioProperties[i];
			var result = voxlQvioServer[prop];
			if (result == 'True' || result == 'true' || result == 'false'|| result == 'False' || result == 'error' || result == false || result == true ) {
				passed = result;
			}
			else {
				passed = 'True';
			}
			healthCheckWriteTestRow(prop,result,passed);
		}
	}
		
		// The last section is the vision hub
	var voxlVisionHub = healthData['voxl-vision-hub']
	if (voxlVisionHub) {
		tableWriteHeaderRow('health-check-table',"---------------- VOXL-Vision-Hub ----------------")
		
		var visionProperties = Object.keys(voxlVisionHub);

		var numProps = visionProperties.length;

		for (var i =0; i < numProps; i++) {
			var prop = visionProperties[i];
			var result = voxlVisionHub[prop];
			if (result == 'True' || result == 'true' || result == 'false'|| result == 'False' || result == 'error' || result == false || result == true ) {
				passed = result;
			}
			else {
				passed = 'True';
			}
			healthCheckWriteTestRow(prop,result,passed);
		}
	}
	healthCheckWriteTopRow(check);
}



// retrieves healthcheck data and calls the print results function

healthChecks = function(){

	
	$.ajax({
		url: 'api/v1/platform/health-check',
		headers: { 'Content-Type': 'application/json' },
		success : function (healthData) {
			healthCheckPrintResults(healthData);
		},
		error: function(xhr,status,error){
			$('#loading-spinner').hide();
			tableWriteHeaderRow('health-check-table',"XX Error: Unable to run health check",'red');
			
		},
		complete: function(){
			$('#loading-spinner').hide();
		}
	});

}

// checks that the drone has a factory mode before starting the health check
$(document).ready(function() { 
	$('#loading-spinner').show();
	
	
	$.ajax({
		url: 'api/v1/platform/status.json',
		headers: { 'Content-Type': 'application/json' },
		success : function (healthData) {

			if(healthData['factory-mode']){
				healthChecks();
			}
			else {
				$('#loading-spinner').hide();
				tableWriteHeaderRow('health-check-table',"XX Error: Unable to get platform status",'red');
			}
		},
		fail : function(xhr,status,error){
			$('#loading-spinner').hide();
			
			tableWriteHeaderRow('health-check-table','XX Error: ' + error,'red');
			tableWriteHeaderRow('health-check-table','XX Status: ' + status,'red');
		}
		
	});
});