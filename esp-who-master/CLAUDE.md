# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP-WHO is an image processing development platform for Espressif chips (ESP32-S3, ESP32-P4) built on ESP-IDF. It provides examples for human face detection/recognition, pedestrian detection, and QR code recognition using the ESP-DL deep learning framework.

**Key technology stack:**
- ESP-IDF v5.4 or v5.5 (required)
- C++ codebase with FreeRTOS task management
- ESP-DL for deep learning models
- LVGL for graphical UI
- Component-based architecture using ESP-IDF component system

## Build System Commands

### Environment Setup (REQUIRED)

Before building any example, you **must** set the `IDF_EXTRA_ACTIONS_PATH` environment variable:

**Linux/macOS:**
```bash
export IDF_EXTRA_ACTIONS_PATH=/path/to/esp-who/tools/
```

**Windows PowerShell:**
```powershell
$Env:IDF_EXTRA_ACTIONS_PATH="/path/to/esp-who/tools/"
```

**Windows CMD:**
```cmd
set IDF_EXTRA_ACTIONS_PATH=/path/to/esp-who/tools/
```

Verify with `echo $IDF_EXTRA_ACTIONS_PATH` (Linux/macOS) or `echo %IDF_EXTRA_ACTIONS_PATH%` (Windows CMD).

### Configure and Build

Navigate to an example directory (e.g., `examples/human_face_recognition/`):

**1. Set target and BSP configuration:**
```bash
idf.py -DSDKCONFIG_DEFAULTS=sdkconfig.bsp.<bsp_name> set-target <esp32xx>
```

**PowerShell (use quotes):**
```powershell
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.bsp.<bsp_name>" set-target "esp32xx"
```

**Available BSP configurations:**
- `human_face_recognition` example: `esp32_s3_eye`, `esp32_s3_korvo_2`, `esp32_p4_function_ev_board`, `lichuang`
- `object_detect` example: `esp32_s3_eye_noglib`, `esp32_s3_korvo_2_noglib`, `esp32_p4_function_ev_board_noglib`
- `qrcode_recognition` example: similar to face recognition BSPs

**Targets:** `esp32s3` for ESP32-S3 boards, `esp32p4` for ESP32-P4 boards

**2. For object_detect example, also specify the detection model:**
```bash
idf.py -DSDKCONFIG_DEFAULTS=sdkconfig.bsp.<bsp_name> -DDETECT_MODEL=<model_name> set-target <esp32xx>
```

Available models: `human_face_detect`, `pedestrian_detect`, `cat_detect`, `dog_detect`

**3. Optional - Configure additional options:**
```bash
idf.py menuconfig
```

**4. Build and flash:**
```bash
idf.py build
idf.py -p <PORT> flash monitor
```

If no port specified, all ports will be scanned.

## High-Level Architecture

### Component Structure

The codebase is organized into reusable components in the `components/` directory:

**Core Task Management:**
- `who_task/`: FreeRTOS task wrapper with pause/resume/stop lifecycle management
  - `WhoTaskBase`: Base task class with event-based state management
  - `WhoTask`: Extended task with yield-to-idle support for power management
  - `WhoTaskGroup`: Manages groups of tasks with coordinated lifecycle
  - All tasks use event groups for synchronization (TASK_STOPPED, TASK_PAUSED, TASK_STOP, TASK_PAUSE, TASK_RESUME)

**Frame Capture Pipeline:**
- `who_frame_cap/`: Asynchronous frame capture and processing pipeline
  - `WhoFrameCap`: Pipeline manager that chains nodes together via FreeRTOS queues
  - `WhoFrameCapNode`: Base class for pipeline nodes (extends `WhoTask`)
  - `WhoFetchNode`: Fetches frames from camera
  - `WhoDecodeNode`: Decodes JPEG/compressed frames to RGB
  - `WhoPPAResizeNode`: Hardware-accelerated resize on ESP32-P4 (uses PPA peripheral)
  - Each node has a ring buffer to handle asynchronous processing
  - Nodes signal downstream subscribers via event groups when new frames are ready

**Detection and Recognition:**
- `who_detect/`: Wrapper for ESP-DL object detection models
- `who_recognition/`: Face recognition with enrollment database management
- `who_qrcode/`: QR code scanning functionality

**Display:**
- `who_frame_lcd_disp/`: LCD display management with double buffering

**Peripherals:**
- `who_peripherals/who_cam/`: Camera drivers (DVP for S3, MIPI-CSI for P4, UVC for USB cameras)
- `who_peripherals/who_lcd/`: LCD drivers
- `who_peripherals/who_usb/`: USB device/host support
- `who_peripherals/who_spiflash_fatfs/`: SPI flash FAT filesystem for face database

**Applications:**
- `who_app/`: High-level application logic
  - `who_app_common/`: Base app classes and result handlers
  - `who_detect_app/`: Object detection apps (LCD and terminal output)
  - `who_recognition_app/`: Face recognition apps with enrollment UI
  - `who_qrcode_app/`: QR code recognition apps

**BSP (Board Support Package):**
- `bsp_lichuang/`: Custom BSP for Lichuang development board
- Other BSPs are pulled from ESP-BSP as managed components

### Example Application Flow

Each example follows this pattern:

1. **Initialize BSP** (`bsp_board_init()`) - sets up peripherals
2. **Mount storage** (SPIFFS/SD card) if needed for model files or face database
3. **Create frame capture pipeline** - different pipelines based on target chip:
   - ESP32-S3: `get_dvp_frame_cap_pipeline()` (DVP camera → Fetch)
   - ESP32-P4: `get_mipi_csi_frame_cap_pipeline()` (MIPI-CSI camera → Fetch)
   - ESP32-P4 USB: `get_uvc_frame_cap_pipeline()` (UVC camera → Fetch → Decode → optional PPA resize)
4. **Create application** (e.g., `WhoRecognitionAppLCD`, `WhoDetectAppLCD`) with pipeline
5. **Run application** - starts all tasks in the pipeline and app

### Pipeline Architecture

Frames flow through a **node-based pipeline** where each node is a FreeRTOS task:

```
Camera → FetchNode → [DecodeNode] → [PPAResizeNode] → App (Detection/Recognition)
            ↓            ↓                ↓
        RingBuffer   RingBuffer      RingBuffer
```

- Nodes are connected via **FreeRTOS queues** (1 frame deep)
- Each node maintains a **ring buffer** of recent frames
- Camera and models run **asynchronously** for higher FPS
- Nodes signal new frames via event bits to subscriber tasks
- `WhoFrameCap::add_node<T>()` template method chains nodes together

### Key Design Patterns

**Task Lifecycle Management:**
All tasks inherit from `WhoTask` and support:
- `run()`: Start task on specific core with priority
- `pause()` / `resume()`: Temporarily pause processing
- `stop()`: Gracefully stop task
- `cleanup()`: Virtual method for resource cleanup

**BSP Selection:**
The build system (via `tools/bsp_ext.py`) validates BSP and target compatibility, then automatically edits `main/idf_component.yml` to pull the correct managed components (BSP package, detection models).

**Memory Management:**
- Frame buffers are reference-counted and recycled
- Ring buffers prevent memory fragmentation
- Models are allocated after BSP init to reduce fragmentation

## Important Implementation Notes

### Camera Frame Synchronization

When displaying detection results on LCD, ring buffer sizing is critical:
- Ring buffer length must cover the model inference time
- If `ringbuf_len = 3`, the displayed frame is 2 frames behind the detection input
- The model must complete inference within 2 frames, or results will lag behind display
- Recommended: `cam_fb_count = MODEL_TIME + 3` where `MODEL_TIME` is frames needed for inference

### Platform-Specific Code

Use conditional compilation based on target:
```cpp
#if CONFIG_IDF_TARGET_ESP32S3
    // DVP camera code
#elif CONFIG_IDF_TARGET_ESP32P4
    // MIPI-CSI or PPA-specific code
#endif
```

### Component Dependencies

Examples use `EXTRA_COMPONENT_DIRS` in `CMakeLists.txt` to reference shared components from `../../components/`. Managed components (ESP-DL, BSP packages, detection models) are automatically fetched via the IDF component manager when you run `set-target`.

### Face Recognition Database

Face embeddings are stored in:
- SPI Flash (FAT filesystem via `who_spiflash_fatfs`)
- SD Card (FAT filesystem via BSP)
- Selection via menuconfig: `CONFIG_DB_FATFS_FLASH` or `CONFIG_DB_FATFS_SDCARD`

## Supported Hardware

- **ESP32-S3-EYE**: DVP camera, LCD (ST7789), SD card, IMU
- **ESP32-S3-Korvo-2**: DVP camera, LCD (ILI9341), audio codec, touch, SD card
- **ESP32-P4-Function-EV-Board**: MIPI-CSI camera, large LCD, audio codec, touch, SD card
- **Lichuang Development Board**: Custom BSP with specific peripheral configuration

## External Dependencies

Models and BSP packages are managed via `idf_component.yml`:
- `espressif/esp-dl`: Deep learning framework
- `espressif/human_face_detect`: Face detection models
- `espressif/human_face_recognition`: Face recognition models
- `espressif/pedestrian_detect`: Pedestrian detection models
- Board-specific BSP packages (e.g., `esp32_s3_eye`, `esp32_p4_function_ev_board`)
- `lvgl/lvgl`: Graphics library (^8.3.3)
