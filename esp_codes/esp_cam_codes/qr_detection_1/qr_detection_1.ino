#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

// =====================================================
// WIFI
// =====================================================

const char* WIFI_SSID = "Manashvi";
const char* WIFI_PASSWORD = "12345678";

// =====================================================
// WEB SERVER
// =====================================================

WebServer server(80);

// =====================================================
// AI-THINKER ESP32-CAM PIN CONFIGURATION
// =====================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM      26
#define SIOC_GPIO_NUM      27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// =====================================================
// HTML PAGE
// =====================================================

const char INDEX_HTML[] PROGMEM = R"HTML(

<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
      content="width=device-width, initial-scale=1.0">

<title>ESP32-CAM QR Scanner</title>


<style>

body
{
    margin: 0;
    padding: 20px;

    background: #111;

    color: white;

    font-family: Arial, sans-serif;

    text-align: center;
}


h1
{
    margin-bottom: 10px;
}


#status
{
    margin: 10px;

    font-size: 18px;
}


#cameraContainer
{
    position: relative;

    display: inline-block;

    max-width: 95vw;
}


#video
{
    display: block;

    width: 320px;

    height: 240px;

    max-width: 95vw;

    background: black;
}


#overlay
{
    position: absolute;

    left: 0;

    top: 0;

    pointer-events: none;
}


#result
{
    margin: 20px auto;

    padding: 15px;

    max-width: 600px;

    background: #222;

    border-radius: 10px;

    word-break: break-all;
}


#qrText
{
    margin-top: 10px;

    font-size: 20px;

    color: #00ff00;
}


</style>

</head>


<body>


<h1>ESP32-CAM QR Scanner</h1>


<div id="status">
    Starting camera...
</div>


<div id="cameraContainer">

    <img
        id="video"
        src="/stream"
    >

    <canvas
        id="overlay">
    </canvas>

</div>


<div id="result">

    <strong>Detected QR:</strong>

    <div id="qrText">
        None
    </div>

</div>


<!-- QR decoder running in the laptop browser -->

<script src="https://cdn.jsdelivr.net/npm/jsqr@1.4.0/dist/jsQR.js"></script>


<script>

// =====================================================
// ELEMENTS
// =====================================================

const video =
    document.getElementById("video");

const overlay =
    document.getElementById("overlay");

const ctx =
    overlay.getContext("2d");

const statusText =
    document.getElementById("status");

const qrText =
    document.getElementById("qrText");


// =====================================================
// HIDDEN CANVAS
// Used to analyze camera frames
// =====================================================

const scanCanvas =
    document.createElement("canvas");

const scanCtx =
    scanCanvas.getContext("2d");


// =====================================================
// LAST QR
// Prevent sending the same QR repeatedly
// =====================================================

let lastQR = "";

let lastSendTime = 0;


// =====================================================
// CAMERA LOADED
// =====================================================

video.onload = function()
{
    statusText.innerText =
        "Camera connected";

    startScanning();
};


// =====================================================
// QR SCANNER
// =====================================================

function startScanning()
{
    scanQR();
}


function scanQR()
{

    // -------------------------------------------------
    // Make sure camera has a frame
    // -------------------------------------------------

    if (
        video.naturalWidth === 0 ||
        video.naturalHeight === 0
    )
    {
        setTimeout(scanQR, 500);

        return;
    }


    const width =
        video.naturalWidth;

    const height =
        video.naturalHeight;


    // -------------------------------------------------
    // Set canvas sizes
    // -------------------------------------------------

    overlay.width = width;

    overlay.height = height;

    scanCanvas.width = width;

    scanCanvas.height = height;


    // -------------------------------------------------
    // Copy current video frame
    // -------------------------------------------------

    scanCtx.drawImage(
        video,
        0,
        0,
        width,
        height
    );


    // -------------------------------------------------
    // Get pixels
    // -------------------------------------------------

    const imageData =
        scanCtx.getImageData(
            0,
            0,
            width,
            height
        );


    // -------------------------------------------------
    // Decode QR
    // -------------------------------------------------

    const code =
        jsQR(
            imageData.data,
            width,
            height,
            {
                inversionAttempts:
                    "attemptBoth"
            }
        );


    // -------------------------------------------------
    // Clear old green box
    // -------------------------------------------------

    ctx.clearRect(
        0,
        0,
        width,
        height
    );


    // =================================================
    // QR FOUND
    // =================================================

    if (code)
    {

        statusText.innerText =
            "QR CODE DETECTED";


        // ---------------------------------------------
        // DRAW GREEN QR BOX
        // ---------------------------------------------

        ctx.beginPath();


        ctx.moveTo(
            code.location.topLeftCorner.x,
            code.location.topLeftCorner.y
        );


        ctx.lineTo(
            code.location.topRightCorner.x,
            code.location.topRightCorner.y
        );


        ctx.lineTo(
            code.location.bottomRightCorner.x,
            code.location.bottomRightCorner.y
        );


        ctx.lineTo(
            code.location.bottomLeftCorner.x,
            code.location.bottomLeftCorner.y
        );


        ctx.closePath();


        ctx.lineWidth = 5;

        ctx.strokeStyle =
            "#00ff00";

        ctx.stroke();


        // ---------------------------------------------
        // GREEN LABEL
        // ---------------------------------------------

        ctx.fillStyle =
            "#00ff00";

        ctx.font =
            "18px Arial";


        ctx.fillText(
            "QR DETECTED",
            code.location.topLeftCorner.x,
            Math.max(
                20,
                code.location.topLeftCorner.y - 8
            )
        );


        // ---------------------------------------------
        // DISPLAY PAYLOAD
        // ---------------------------------------------

        qrText.innerText =
            code.data;


        // ---------------------------------------------
        // SEND PAYLOAD TO ESP32
        // ---------------------------------------------

        const now =
            Date.now();


        if (
            code.data !== lastQR ||
            now - lastSendTime > 3000
        )
        {

            lastQR =
                code.data;

            lastSendTime =
                now;


            sendQRToESP32(
                code.data
            );
        }

    }

    else
    {

        statusText.innerText =
            "Scanning for QR code...";

    }


    // -------------------------------------------------
    // Scan again
    // -------------------------------------------------

    setTimeout(
        scanQR,
        100
    );
}


// =====================================================
// SEND QR DATA BACK TO ESP32
// =====================================================

function sendQRToESP32(data)
{

    fetch(
        "/qr?data=" +
        encodeURIComponent(data)
    )

    .then(
        response =>
        {
            console.log(
                "QR sent to ESP32"
            );
        }
    )

    .catch(
        error =>
        {
            console.log(
                "Could not send QR:",
                error
            );
        }
    );
}

</script>


</body>

</html>

)HTML";


// =====================================================
// ROOT PAGE
// =====================================================

void handleRoot()
{
    server.send(
        200,
        "text/html",
        INDEX_HTML
    );
}


// =====================================================
// QR RECEIVED FROM BROWSER
// =====================================================

void handleQR()
{
    if (!server.hasArg("data"))
    {
        server.send(
            400,
            "text/plain",
            "Missing QR data"
        );

        return;
    }


    String qrData =
        server.arg("data");


    Serial.println();
    Serial.println("==============================");
    Serial.println("QR CODE DETECTED");
    Serial.println("==============================");

    Serial.print(
        "Payload: "
    );

    Serial.println(
        qrData
    );

    Serial.println("==============================");
    Serial.println();


    server.send(
        200,
        "text/plain",
        "QR received"
    );
}


// =====================================================
// CAMERA STREAM
// =====================================================

void handleStream()
{
    WiFiClient client =
        server.client();


    client.println(
        "HTTP/1.1 200 OK"
    );

    client.println(
        "Content-Type: multipart/x-mixed-replace; boundary=frame"
    );

    client.println(
        "Cache-Control: no-cache"
    );

    client.println(
        "Access-Control-Allow-Origin: *"
    );

    client.println();


    while (client.connected())
    {

        // ---------------------------------------------
        // Capture image
        // ---------------------------------------------

        camera_fb_t *fb =
            esp_camera_fb_get();


        if (!fb)
        {
            Serial.println(
                "Camera capture failed!"
            );

            break;
        }


        // ---------------------------------------------
        // Send JPEG frame
        // ---------------------------------------------

        client.printf(
            "--frame\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n",
            fb->len
        );


        client.write(
            fb->buf,
            fb->len
        );


        client.print(
            "\r\n"
        );


        // ---------------------------------------------
        // Return frame buffer
        // ---------------------------------------------

        esp_camera_fb_return(
            fb
        );


        delay(50);
    }
}


// =====================================================
// CAMERA INITIALIZATION
// =====================================================

bool initCamera()
{

    camera_config_t config;


    config.ledc_channel =
        LEDC_CHANNEL_0;

    config.ledc_timer =
        LEDC_TIMER_0;


    config.pin_d0 =
        Y2_GPIO_NUM;

    config.pin_d1 =
        Y3_GPIO_NUM;

    config.pin_d2 =
        Y4_GPIO_NUM;

    config.pin_d3 =
        Y5_GPIO_NUM;

    config.pin_d4 =
        Y6_GPIO_NUM;

    config.pin_d5 =
        Y7_GPIO_NUM;

    config.pin_d6 =
        Y8_GPIO_NUM;

    config.pin_d7 =
        Y9_GPIO_NUM;


    config.pin_xclk =
        XCLK_GPIO_NUM;

    config.pin_pclk =
        PCLK_GPIO_NUM;

    config.pin_vsync =
        VSYNC_GPIO_NUM;

    config.pin_href =
        HREF_GPIO_NUM;


    config.pin_sccb_sda =
        SIOD_GPIO_NUM;

    config.pin_sccb_scl =
        SIOC_GPIO_NUM;


    config.pin_pwdn =
        PWDN_GPIO_NUM;

    config.pin_reset =
        RESET_GPIO_NUM;


    config.xclk_freq_hz =
        20000000;


    // ---------------------------------------------
    // JPEG
    // ---------------------------------------------

    config.pixel_format =
        PIXFORMAT_JPEG;


    // ---------------------------------------------
    // QVGA = 320 x 240
    // Good starting point for this board
    // ---------------------------------------------

    config.frame_size =
        FRAMESIZE_QVGA;


    // ---------------------------------------------
    // JPEG quality
    // Lower number = better quality
    // ---------------------------------------------

    config.jpeg_quality =
        12;


    // ---------------------------------------------
    // One framebuffer
    // Keeps memory usage lower
    // ---------------------------------------------

    config.fb_count =
        1;


    // ---------------------------------------------
    // Use internal RAM
    // Important because your board previously
    // reported a PSRAM memory test failure.
    // ---------------------------------------------

    config.fb_location =
        CAMERA_FB_IN_DRAM;


    // ---------------------------------------------
    // Initialize
    // ---------------------------------------------

    Serial.println(
        "Initializing camera..."
    );


    esp_err_t err =
        esp_camera_init(
            &config
        );


    if (err != ESP_OK)
    {

        Serial.print(
            "Camera initialization failed: 0x"
        );

        Serial.println(
            err,
            HEX
        );

        return false;
    }


    Serial.println(
        "Camera initialized successfully!"
    );


    return true;
}


// =====================================================
// SETUP
// =====================================================

void setup()
{

    Serial.begin(
        115200
    );


    delay(1000);


    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "ESP32-CAM QR WEB SCANNER"
    );

    Serial.println(
        "================================"
    );


    // -------------------------------------------------
    // CAMERA
    // -------------------------------------------------

    if (!initCamera())
    {

        Serial.println(
            "Camera failed. Halting."
        );

        while (true)
        {
            delay(1000);
        }
    }


    // -------------------------------------------------
    // WIFI
    // -------------------------------------------------

    Serial.println();
    Serial.println(
        "Connecting to WiFi..."
    );


    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );


    while (
        WiFi.status() != WL_CONNECTED
    )
    {

        delay(500);

        Serial.print(".");
    }


    Serial.println();

    Serial.println(
        "WiFi connected!"
    );


    // -------------------------------------------------
    // IP
    // -------------------------------------------------

    Serial.print(
        "ESP32-CAM IP address: "
    );

    Serial.println(
        WiFi.localIP()
    );


    // -------------------------------------------------
    // WEB ROUTES
    // -------------------------------------------------

    server.on(
        "/",
        HTTP_GET,
        handleRoot
    );


    server.on(
        "/stream",
        HTTP_GET,
        handleStream
    );


    server.on(
        "/qr",
        HTTP_GET,
        handleQR
    );


    // -------------------------------------------------
    // START SERVER
    // -------------------------------------------------

    server.begin();


    Serial.println(
        "Web server started!"
    );


    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.print(
        "OPEN THIS IN YOUR BROWSER: "
    );

    Serial.print(
        "http://"
    );

    Serial.print(
        WiFi.localIP()
    );

    Serial.println(
        "/"
    );

    Serial.println(
        "================================"
    );
}


// =====================================================
// LOOP
// =====================================================

void loop()
{

    server.handleClient();

    delay(5);
}