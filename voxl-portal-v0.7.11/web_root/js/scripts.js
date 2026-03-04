// Directory where JSON files are stored
const serverAddress = 'http://192.168.0.99/';
const jsonDirectory = serverAddress;

// Fetch the list of JSON files from the server directory
async function listJsonFiles() {
    try {
        const response = await fetch(jsonDirectory);
        if (!response.ok) throw new Error(`Failed to list files. Status: ${response.status}`);
        
        const text = await response.text();
        const parser = new DOMParser();
        const htmlDoc = parser.parseFromString(text, 'text/html');
        const links = Array.from(htmlDoc.querySelectorAll('a'));

        const fileSelector = document.getElementById('file-selector');
        fileSelector.innerHTML = '<option value="" disabled selected>Select a JSON file</option>';
        
        links.forEach(link => {
            const fileName = link.textContent;
            if (fileName.endsWith('.json')) {
                const option = document.createElement('option');
                option.value = fileName;
                option.textContent = fileName;
                fileSelector.appendChild(option);
            }
        });

        if (fileSelector.length === 1) {
            alert("No JSON files found in the directory.");
        }
    } catch (error) {
        console.error('Error listing JSON files:', error);
        alert('Error listing JSON files: ' + error.message);
    }
}

// Fetch and display the selected JSON file
async function fetchSelectedFile() {
    const fileSelector = document.getElementById('file-selector');
    const selectedFile = fileSelector.value;

    if (!selectedFile) {
        alert('Please select a file');
        return;
    }

    try {
        const fileUrl = jsonDirectory + selectedFile;
        const response = await fetch(fileUrl);
        if (!response.ok) throw new Error('Failed to fetch the file');
        
        const data = await response.json();
        const jsonOutput = document.getElementById('json-output');
        jsonOutput.textContent = JSON.stringify(data, null, 4);
    } catch (error) {
        console.error('Error fetching JSON:', error);
        document.getElementById('json-output').textContent = 'Error loading JSON data.';
    }
}

// Load the file list on page load
window.onload = listJsonFiles;
