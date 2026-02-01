import { WebAPIClient } from "../webAPIClient.js";
import { ModalControl, SpinnerControl, NavMenuControl } from "../UIControl.js";

export class Settings{
    constructor () {
        this.webClient = new WebAPIClient();

        this.fieldset = document.getElementById("fieldset");

        this.linearSpd = document.getElementById("linearSpd");
        this.labelLinear = document.getElementById("labelLinear");

        this.angularSpd = document.getElementById("angularSpd");
        this.labelAngular = document.getElementById("labelAngular");

        this.navSet = document.getElementById("rdoNav");
        this.slamSet = document.getElementById("rdoSlam");

        this.btnResolutionDropdown = document.getElementById("targetResolution");
        this.resolutionList = document.getElementById("resolutionList");

        this.btnRestart = document.getElementById("restart");
        this.restartSpinner = new SpinnerControl(this.btnRestart);
        this.btnRestart.onclick = this.restart.bind(this);

        this.btnSave = document.getElementById("save");
        this.saveSpinner = new SpinnerControl(this.btnSave);
        this.btnSave.onclick = this.save.bind(this);

        this.linearSpd.oninput = this.onchangeSpeedLinear.bind(this);
        this.angularSpd.oninput = this.onchangeSpeedAngular.bind(this);

        this.btnResolutionDropdown.onclick = this.onclickDropdown.bind(this);


        this.currentLinearSpeed = 0;
        this.currentAngularSpeed = 0;
        this.resolutions = ["640x480", "800x600", "1280x720", "1920x1080"];
        this.currentRes = "";

        this.dropdownOpen = false;
        this.modalControl = null;

    }

    async onInit()
    {
        let data = await this.webClient.loadSettings();
        console.log(data);

        this.linearSpd.value = data.Linear;
        this.onchangeSpeedLinear();

        this.angularSpd.value = data.Angular;
        this.onchangeSpeedAngular();

        this.navSet.checked = data.SLAM === false;
        this.slamSet.checked = data.SLAM === true;

        this.selectResolution(data.Resolution);

        this.fieldset.disabled = false;        
    }

    onchangeSpeedLinear()
    {
        this.currentLinearSpeed = parseFloat(this.linearSpd.value);
        this.labelLinear.textContent = this.currentLinearSpeed.toFixed(2);
    }

    onchangeSpeedAngular()
    {
        this.currentAngularSpeed = parseFloat(this.angularSpd.value);
        this.labelAngular.textContent = this.currentAngularSpeed.toFixed(2);
    }

    selectResolution(res)
    {
        this.dropdownOpen = false;
        this.showDropdown(false);

        this.currentRes = res;
        this.btnResolutionDropdown.textContent = this.currentRes;
    }

    showDropdown(show)
    {
        if (show)
            this.resolutionList.classList.add("show"); 
        else
            this.resolutionList.classList.remove("show"); 
    }

    onclickDropdown()
    {
        this.dropdownOpen = !this.dropdownOpen;

        if (this.dropdownOpen)
        {
            this.resolutionList.innerHTML = this.resolutions.map((res) => `<li><a class="dropdown-item" onclick='window.settingsPage.selectResolution("${res}")' href="#">${res}</a></li>`).join("");
        }             
        this.showDropdown(this.dropdownOpen);
    }
    
    async save() {

        let useSlam = this.slamSet.checked;
        this.fieldset.disabled = true;

        const obj = {
            Linear: this.currentLinearSpeed,
            Angular: this.currentAngularSpeed,
            Resolution: this.currentRes,
            SLAM: useSlam
        };

        this.saveSpinner.setSpin("Saving...");

        try {
            await this.webClient.saveSettings(obj);
        } 
        finally {
            // restore button
            this.fieldset.disabled = false;
            this.saveSpinner.stopSpin();
        }
    }

    async restart() {

        if (this.modalControl === null)
            this.modalControl = new ModalControl();

        this.modalControl.showPopup();
        this.modalControl.modalOk = async (password) => {

            this.fieldset.disabled = true;
            this.modalControl.removePopup();

            this.restartSpinner.setSpin("Restarting...");

            await this.webClient.requestReset(password);

            alert("Restarted Successfully");
            location.reload();
        };
    }
}


window.settingsPage = new Settings();
await window.settingsPage.onInit();

const nvctrl = new NavMenuControl();
