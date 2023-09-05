window.onload = () => {

    console.log("hello");
    const configJsonEntry = document.getElementById("configJson");
    const resetButton = document.getElementById("resetButton");
    const saveButton = document.getElementById("saveButton");
    
    // Hook up button events
    resetButton.onclick = (e) => {
        e.preventDefault();
        console.log("Reset Button Clicked");
        fetch("/resetconfig", {
            method: "post"
        })
        .then(() => {
            loadConfig();
        });
    };
    
    saveButton.onclick = (e) => {
        e.preventDefault();
        console.log("Save Button Clicked");
    
        try {
            JSON.parse(configJsonEntry.value);
            fetch("/saveconfig", {
                body: configJsonEntry.value,
                headers: {
                    "Content-Type": "text/json"
                },
                method: "post"
            })
            .then(() => {
                alert("Configuration Updated. Please reset the XRP");
            })
        }
        catch (e) {
            alert("Invalid JSON. Please check and try again");
        }
    }
    
    // load data and then enable the buttons
    loadConfig();
    
    function loadConfig() {
        fetch("/getconfig")
        .then((response) => response.json())
        .then((xrpConfigJson) => {
            configJsonEntry.value = JSON.stringify(xrpConfigJson, null, 4);
        })
        .catch((err) => {
            console.log("ERROR");
            console.log(err);
        });
    }
    
    };