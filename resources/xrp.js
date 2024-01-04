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
            loadConfig()
            .then(() => {
                alert("Configuration reset to default. Please reset the XRP");
            });
        });
    };
    
    saveButton.onclick = (e) => {
        e.preventDefault();
        console.log("Save Button Clicked");
        
        try {
            // Check if can parse
            let jsonObj = JSON.parse(configJsonEntry.value);
            
            // Check if Defaul AP exists and has password 8 characters or more
            if(jsonObj["network"] && jsonObj["network"]["defaultAP"] && jsonObj["network"]["defaultAP"]["password"]) {
                let password = jsonObj["network"]["defaultAP"]["password"];
                if(password.length < 8 || password.length > 63) {
                    throw new Error("Default AP password must be at least 8 characters and less than 64 per WPA standards");
                }
            } else {
                throw new Error("In \"network\", must have field \"defaultAP\": {\"ssid\":\"APname\", \"password\":\"mypassword\"}");
            }

            // Don't allow for escape characters
            if(configJsonEntry.value.toString().includes("\\")) {
                throw new Error("Cannot use escape character '\\'.");
            }

            // Save config back to XRP disk
            fetch("/saveconfig", {
                body: configJsonEntry.value,
                headers: {
                    "Content-Type": "text/json"
                },
                method: "post"
            })
            .then(() => {
                alert("Configuration Updated. Please reset the XRP");
            });
        }
        catch (e) {
            alert("Invalid JSON. Please check and try again.\n" + e.message);
        }
    }
    
    // load data and then enable the buttons
    loadConfig();
    
    function loadConfig() {
        return fetch("/getconfig")
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