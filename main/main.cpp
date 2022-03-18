#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Helper.h"
#include "CImageBasis.h"
#include "CRotateImage.h"
#include "CImageCut.h"
#include "stb_image.h"
#include "stb_image_write.h"
#include "stb_image_resize.h"

#include "CTfLiteClass.h"
#include "Helper.h"
#include "connect_wlan.h"
#include "server_main.h"

//#include "dig-01.h"

#include "esp_camera.h"

static const char *TAG = "ESP32CAM-CNN";

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////// 0) Konfiguration und Einstellungen /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
    // WLan Einstellungen
    const char* WLAN_ssid     = "SoJoFritzBox";
    const char* WLAN_password = "z1504jlp77";

    // Bildeinstellungen Bildgröße = QVGA (320x240x3 Farben)
    int Camera_Image_Size_x = 320;
    int Camera_Image_Size_y = 240;
    int Camera_Channels = 3;

    // ROI Einstellungen
    // Das ROI definiert den Ausschnitt aus dem Kamerabild, der zunächst ausgeschnitten
    // und anschließend auf die Inputgröße des neuronalen Netzes reskaliert wird.
    bool ROI_rotate = false;             // true: Drehung des ROIs um 90°
    int ROI_x = 20;
    int ROI_y = 20;
    int ROI_dx = 136;
    int ROI_dy = 205;

    // Dateinamen zum Speichern der Bilder auf der SD-Karte
    std::string image_name_input  = "/sdcard/original.jpg";
    std::string image_name_ROI    = "/sdcard/roi.jpg";
    std::string image_name_resize = "/sdcard/resize.bmp";

    // Einstellungen für das neuronale Netz
    std::string cnn_tflite = "/sdcard/dig-01.tfl";
    int cnn_input_x = 20;
    int cnn_input_y = 32;


    // Kamerakonfiguration (GPIO-Belegung, Timer, Bildformat / -größe)
    static camera_config_t camera_config = {
        .pin_pwdn = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sscb_sda = 26,
        .pin_sscb_scl = 27,
        .pin_d7 = 35,
        .pin_d6 = 34,
        .pin_d5 = 39,
        .pin_d4 = 36,
        .pin_d3 = 21,
        .pin_d2 = 19,
        .pin_d1 = 18,
        .pin_d0 = 5,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG, // ACHTUNG: JPEG notwendig
        .frame_size = FRAMESIZE_QVGA,   // ACHTUNG: bei Veränderung muss auch die Bildgröße weiter oben angepasst werden!   
        .jpeg_quality = 5, 
        .fb_count = 1,      
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };
/////////// Ende Konfig ////////////////////////////////////////////////////////////////////////////


extern "C" void app_main()
{
    // Initialisiere Hardware
    if (!Init_SDCard_GPIO())
        return;
    CheckPSRAM();
    Init_NVS();

    if (esp_camera_init(&camera_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
    printf("\n");

    // Verbinde mit WLAN und Starte den http-Server
    wifi_init_sta(WLAN_ssid, WLAN_password);

    printf("Start Server ...\n");
    server =  start_webserver();
    register_server_main_uri(server, "/sdcard");


    // Lade und Initialisiere das neuronale Netz
    printf("Lade und Initialisere neuronales Netz ... \n");
    CTfLiteClass* neuralnetwork = new CTfLiteClass;
    if (!neuralnetwork->LoadModelFromFile(cnn_tflite)) return;   // dynamisch

    SetResult("Not-a-Number");

    // Endlosschleife mit der Bildaufnahme, Verarbeitung, Neuronale Berechnung
    while (1)
    {
        printf("IP-Adresse: %s\n", GetIPAdresse().c_str());
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////// 1) Bild von Kamera aufnehmen ///////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        CImageCut ImageCamera(Camera_Image_Size_x, Camera_Image_Size_y, Camera_Channels);

        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *fb = esp_camera_fb_get();
        ImageCamera.LoadFromMemory(fb->buf, fb->len);
        esp_camera_fb_return(fb);

        ImageCamera.drawRect(ROI_x-1, ROI_y-1, ROI_dx+2, ROI_dy+2, 255, 0, 0);
        UnLockFile(true);
        ImageCamera.SaveToFile("/sdcard/original.jpg");
        UnLockFile(false);

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////// 2) ROI Bildgröße ändern und abspeichern ////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        CImageBasis ImageROI(ROI_dx, ROI_dy, Camera_Channels);

        ESP_LOGI(TAG, "Cut ROI and save to file...");
        ImageCamera.CutAndSave(ROI_x, ROI_y, ROI_dx, ROI_dy, &ImageROI);
        UnLockFile(true);
        ImageROI.SaveToFile(image_name_ROI);
        UnLockFile(false);

        ESP_LOGI(TAG, "Rotate and Resize ROI ...");
        CRotateImage ImageInputCNN(&ImageROI);
        if (ROI_rotate)
            ImageInputCNN.Rotate(90);
        ImageInputCNN.Resize(cnn_input_x, cnn_input_y);

        UnLockFile(true);
        ImageInputCNN.SaveToFile(image_name_resize);
        UnLockFile(false);


        ////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////// Verkleinertes Bild durch TFlite schicken ///////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        int result;

        printf("Lade Bilddaten...\n");
        UnLockFile(true);
        neuralnetwork->LoadInputImage(image_name_resize.c_str());
        UnLockFile(false);
        neuralnetwork->Invoke();
        result = neuralnetwork->GetOutClassification();
    
        printf("  CNN-Klassifizierung: %d\n", (int) result);
        if (result == 10)
            SetResult("Not-a-Number");
        else
            SetResult(to_string(result));


        TickType_t xDelay = 100 / portTICK_PERIOD_MS;
        printf("main: sleep for : %ldms\n", (long) xDelay*10);
        vTaskDelay( xDelay ); 
    }

    delete neuralnetwork;       // Freigeben des Speichers
}
