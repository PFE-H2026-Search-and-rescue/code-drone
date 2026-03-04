
// will store specific calibration file data
let calibrationValues = {}


// prints out the inspect services data to a table
// gets data from ajax call at bottom
function servicePrint(serviceData){
	serviceWriteTopTest();
	var servData = serviceData['Services']
	var services = Object.keys(servData)
	var numServices = services.length
	for (var i = 0; i < numServices; i++){
		var service = services[i];
		var enabled = servData[service]['Enabled'];
		var running = servData[service]['Running'];
		var cpu = servData[service]['Cpu_usage'];
		
		serviceWriteTestRow(service,enabled,running,cpu);
		
	}

}
// function which will print out a full (one column) row to the table specified with a color parameter (default black)
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

// function that writes a test row to the table for inspect Services
function serviceWriteTestRow(service,enabled,running,cpu){
	var tableBody = document.getElementById('inspect-services-table').getElementsByTagName('tbody')[0];
    var row = document.createElement('tr');
    var cell1 = document.createElement('td');
    cell1.innerHTML = service;
    row.appendChild(cell1);
	var cell2 = document.createElement('td');
	cell2.innerHTML = enabled
	if (enabled == 'Enabled'){
		cell2.style.color = 'green'
	}
	else {
		cell2.style.color = 'red'
	}
	row.appendChild(cell2);
	var cell3 = document.createElement('td');
	cell3.innerHTML = running
	if (running != 'Not Running'){
		cell3.style.color = 'green'
	}
	else {
		cell3.style.color = 'red'
	}
	row.appendChild(cell3);
	var cell4 = document.createElement('td');
	if (cpu == null) {
		cell4.innerHTML = '0.0 %'
		cell4.style.color = 'green'
	}
	else {
		cell4.innerHTML = cpu + ' %'
		if (cpu > 20.0){
			cell4.style.color = 'orange'
		}
		else if (cpu > 50.0){
			cell4.style.color = 'red'
		}
		else{
			cell4.style.color = 'green'
		}
	}
	row.appendChild(cell4);
    tableBody.appendChild(row);


}

//writes the labels for the columns of the table

function serviceWriteTopTest(){
	var tableBody = document.getElementById('inspect-services-table').getElementsByTagName('tbody')[0];
    var row = document.createElement('tr');
    var cell1 = document.createElement('td');
    cell1.innerHTML = '<b>Service</b>';
	cell1.style.color = 'blue';
    row.appendChild(cell1);
	var cell2 = document.createElement('td');
	cell2.innerHTML = '<b>Enabled</b>'
	cell2.style.color = 'blue'
	row.appendChild(cell2);
	var cell3 = document.createElement('td');
	cell3.innerHTML = '<b>Running</b>'
	cell3.style.color = 'blue'
	row.appendChild(cell3);
	var cell4 = document.createElement('td');
	cell4.innerHTML = '<b>CPU %</b>'
	cell4.style.color = 'blue'
	row.appendChild(cell4);
    tableBody.appendChild(row);
}

// writes a row to the calibration check table
function calibrationWriteTestRow(fileStatus,filePath){
	var tableBody = document.getElementById('check-calibration-table').getElementsByTagName('tbody')[0];
    var row = document.createElement('tr');
    var cell1 = document.createElement('td');
    cell1.innerHTML = fileStatus;
	var cell2 = document.createElement('td');
	cell2.innerHTML = filePath
	if (fileStatus == 'Present'){
		cell1.style.color = 'green'
		cell2.style.color='green'
		row.style.fontWeight = 'bold';
		cell1.style.fontSize = '14px';
        cell2.style.fontSize = '14px';
	}
	else if (fileStatus == 'Missing'){
		cell1.style.color = 'red'
		cell2.style.color='red'
		row.style.fontWeight = 'bold';
		cell1.style.fontSize = 'larger';
        cell2.style.fontSize = 'larger';
	}
	else {
		cell1.style.color = 'black'
		cell2.style.color = 'black'
		cell1.style.width = '50%';
        cell2.style.width = '50%';
		cell1.style.fontSize = '12px';
        cell2.style.fontSize = '12px';
	}
	row.appendChild(cell1);
	row.appendChild(cell2);
	tableBody.appendChild(row);
}

// prints out the calibration check and important data from each calibration file
function calibrationPrint(calibrationData){
	presentData = calibrationData['Present']
	numPresentData = presentData.length
	missingData = calibrationData['Missing']
	numMissingData = missingData.length
	tableWriteHeaderRow('check-calibration-table',' Calibration Files: ')
	if (numPresentData == 0) {
		tableWriteHeaderRow('check-calibration-table',' There are no calibration files present', 'red')
	}
	else {
		for (var i = 0; i < numPresentData; i++){
			
				calibrationWriteTestRow('Present', presentData[i])
				if (presentData[i].includes('voxl-imu-server')) {
					calibrationWriteTestRow('-	gyro0_offset : ', calibrationValues.gyro_offset)
					calibrationWriteTestRow('-	accl0_offset : ', calibrationValues.accl_offset)
					calibrationWriteTestRow('-	accl0_scale : ', calibrationValues.accl_scale)
				}
				else if (presentData[i].includes('opencv_tracking_intrinsics')) {
					calibrationWriteTestRow('-	width : ', calibrationValues.width)
					calibrationWriteTestRow('-	height : ', calibrationValues.height)
					calibrationWriteTestRow('-	distortion_model : ', calibrationValues.distortion_model)
					calibrationWriteTestRow('-	calibration_time : ', calibrationValues.calibration_time)
				}
				else if (presentData[i].includes('parameters_gyro')){
					calibrationWriteTestRow('-	cal_gyro0_xoff', calibrationValues.cal_gyro0_xoff)
					calibrationWriteTestRow('-	cal_gyro0_yoff', calibrationValues.cal_gyro0_yoff)
					calibrationWriteTestRow('-	cal_gyro0_zoff', calibrationValues.cal_gyro0_zoff)
				}
				else if (presentData[i].includes('parameters_acc')){
					calibrationWriteTestRow('-	cal_acc0_xoff', calibrationValues.cal_acc0_xoff)
					calibrationWriteTestRow('-	cal_acc0_yoff', calibrationValues.cal_acc0_yoff)
					calibrationWriteTestRow('-	cal_acc0_zoff', calibrationValues.cal_acc0_zoff)
				}
				else if (presentData[i].includes('parameters_level')){
					calibrationWriteTestRow('-	sens_board_x_off', calibrationValues.sens_board_x_off)
					calibrationWriteTestRow('-	sens_board_y_off', calibrationValues.sens_board_y_off)
				}
				else if (presentData[i].includes('parameters_mag')){
					calibrationWriteTestRow('-	cal_mag0_xscale', calibrationValues.cal_mag0_xscale)
					calibrationWriteTestRow('-	cal_mag0_yscale', calibrationValues.cal_mag0_yscale)
					calibrationWriteTestRow('-	cal_mag0_zscale', calibrationValues.cal_mag0_zscale)
				}
		}
	}
	if (numMissingData == 0){
		tableWriteHeaderRow('check-calibration-table',' All files are present', 'green')
	}
	else {
		for (var i = 0; i < numMissingData; i++){
			calibrationWriteTestRow('Missing', missingData[i])
		}
	}

}

// extracts important data from the string of file data and populates calibrationValues
function extractValuesFromString(dataString) {
    

    if (dataString.includes("gyro0_offset")) {
        
        const regex = /"gyro0_offset":\s*\[([-0-9.,\s]+)\],\s*"accl0_offset":\s*\[([-0-9.,\s]+)\],\s*"accl0_scale":\s*\[([-0-9.,\s]+)\]/;
        const matches = dataString.match(regex);
        if (matches) {
            calibrationValues.gyro_offset = matches[1];
            calibrationValues.accl_offset = matches[2];
            calibrationValues.accl_scale = matches[3];
        }
    } else if (dataString.includes("width")) {
        const regex = /width:\s*(\d+)\s+height:\s*(\d+)\s+distortion_model:\s*(\w+)\s+calibration_time:\s*"([^"]+)"/;
        const matches = dataString.match(regex);
        if (matches) {
            calibrationValues.width = matches[1];
            calibrationValues.height = matches[2];
            calibrationValues.distortion_model = matches[3];
            calibrationValues.calibration_time = matches[4];
        }
    } else {
        const regex = /(CAL_GYRO0_XOFF|CAL_GYRO0_YOFF|CAL_GYRO0_ZOFF|CAL_ACC0_XOFF|CAL_ACC0_YOFF|CAL_ACC0_ZOFF|SENS_BOARD_X_OFF|SENS_BOARD_Y_OFF|CAL_MAG0_XSCALE|CAL_MAG0_YSCALE|CAL_MAG0_ZSCALE)\s*([-0-9.]+)/g;
        let match;
        while ((match = regex.exec(dataString)) !== null) {
            calibrationValues[match[1].toLowerCase()] = match[2];
        }
    }

    
}

// calls the appropriate head/tail commands to extract the wanted lines from the file
function retrieveCalibrationFileData(filePath, callback){	
		if (filePath.includes('imu-server')){
			
			fileUrl = "/api/v1/platform/check-calibration-1";
		}
		else if (filePath.includes('opencv')){
			fileUrl = '/api/v1/platform/check-calibration-2';
		}
		else if (filePath.includes('parameters_gyro')){
			fileUrl = '/api/v1/platform/check-calibration-3';
		}
		else if (filePath.includes('parameters_acc')){
			fileUrl = '/api/v1/platform/check-calibration-4';
		}
		else if (filePath.includes('parameters_level')){
			fileUrl = '/api/v1/platform/check-calibration-5';
		}
		else if (filePath.includes('parameters_mag')){
			fileUrl = '/api/v1/platform/check-calibration-6';
		}

		
		
		$.ajax({
			url: fileUrl,
			datatype: 'text',
			async: true,
			success : function (fileData) {
				


				// send the lines of data to be stripped down to the important values
				extractValuesFromString(fileData)

				// signify that we have finished extracting data from file
				callback();
				
				
				

				
			},
			error: function(xhr,status,error){
				
				tableWriteHeaderRow('check-calibration-table',xhr + " " + status + " " + error,'red')
				callback();
			},
			
			
		});
	
	
}
// calls the retrieve data function for each file
function popValues(calibrationData){
    presentData = calibrationData['Present']
    numPresentData = presentData.length
    var counter = 0; 

    
    function callPrint() {
		
        $('#loading-spinner2').hide();
        calibrationPrint(calibrationData);
    }

    for (var j = 0; j < numPresentData; j++) {
        retrieveCalibrationFileData(presentData[j], function() {
           
            counter++;
            
            if (counter === numPresentData) {
                callPrint();
            }
        });
    }
}

// calls cpp code in portal api to run the voxl-check-calibration command 
function checkCalibration(){
	$.ajax({
		url: 'api/v1/platform/check-calibration',
		headers: { 'Content-Type': 'application/json' },
		async: true,
		success : function (calibrationData) {
			popValues(calibrationData)
			
			

			
		},
		error: function(xhr,status,error){
			
			tableWriteHeaderRow('check-calibration-table',xhr + " " + status + " " + error,'red')
			
		},
		
	});
}
// calls cpp code in portal api to run the voxl-inspect-services command 
function inspectServices(){
	$.ajax({
		url: 'api/v1/platform/inspect-services',
		headers: { 'Content-Type': 'application/json' },
		async: true,
		success : function (serviceData) {
			$('#loading-spinner').hide();
			servicePrint(serviceData);
			

			
		},
		error: function(xhr,status,error){
			
			tableWriteHeaderRow('inspect-services-table',xhr + " " + status + " " + error,'red')
			
		},
		complete: function(){
			
			$('#loading-spinner').hide();
			
			
		}
	});
	
}


$(document).ready(function() { 
    $('#loading-spinner').show();
	$('#loading-spinner2').show();
    inspectServices();
	checkCalibration();
});