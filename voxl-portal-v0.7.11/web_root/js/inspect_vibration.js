
function renderAnsiCodes(text) {
    // Define the mapping of ANSI codes to HTML tags
        var ansiMap = {
            '\u001b[1m': '<span style="font-weight: bold;">',
            '\u001b[31m': '<span style="color: red;">',
            '\u001b[32m': '<span style="color: green;">',
            '\u001b[33m': '<span style="color: yellow;">',
            '\u001b[34m': '<span style="color: blue;">',
            '\u001b[35m': '<span style="color: magenta;">',
            '\u001b[36m': '<span style="color: cyan;">',
            '\u001b[37m': '<span style="color: white;">',
            '\u001b[39m': '<span style="color: inherit;">',
            '\u001b[0m': '</span>'
        };
        
        // Regular expression to match ANSI escape codes
        var ansiRegex = /\u001b\[(?:\d{1,2}(?:;\d{1,2})*)?[m|K]/g;

        // Replace ANSI codes with equivalent HTML tags
        var replacedText = text.replace(ansiRegex, function (match) {
            if (ansiMap.hasOwnProperty(match)) {
                return ansiMap[match];
            } else {
                return '';
            }
        });

        return replacedText;
    }



function printVibrationData(vibrationData){
    vibrationData = vibrationData.replace(/ /g, "&nbsp;&nbsp;");
    coloredData = renderAnsiCodes(vibrationData);
    var display_data = document.getElementById("vibration-data-placeholder");
    display_data.innerHTML = coloredData;
    display_data.style.fontSize = "25px";

    
}

function retrieveVibrationData(){
    $.ajax({
        url: '/api/v1/platform/inspect-vibration',
        datatype: 'text',
        success : function (vibrationData) {
            console.log("retrieved vibration data");
            $('#loading-spinner3').hide();
            printVibrationData(vibrationData);
        },
        error : function(xhr,status,error){
            printVibrationData('Failed to retrieve Vibration Data');
        },
        complete: function(){
            $('#loading-spinner3').hide();
        }
        
    });
}
$(document).ready(function() { 
	$('#loading-spinner3').show();
	retrieveVibrationData();
    setInterval(retrieveVibrationData, 1000);
	
});