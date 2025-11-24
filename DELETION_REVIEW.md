# 🗑️ USELESS FILES REVIEW - PENDING DELETION

**Date**: 2025-11-23
**Purpose**: Identify and document files that can be safely deleted from the codebase

---

## ⚠️ IMPORTANT: REVIEW BEFORE DELETION

**Please review this list carefully before approving deletion.**
Files are categorized by risk level and purpose.

---

## 🔴 HIGH CONFIDENCE - Safe to Delete

### 1. OLD DOCUMENTATION FILES (Redundant/Outdated)
These are old summary/status files that have been superseded by current documentation:

- **`/home/mercleDev/codebase/COMPLETE_IMPLEMENTATION_SUMMARY.txt`**
  - Reason: Outdated summary, superseded by newer docs
  - Size: Text file
  - Risk: LOW - just documentation

- **`/home/mercleDev/codebase/FINAL_FIXES_APPLIED.txt`**
  - Reason: Historical log, no longer needed
  - Size: Text file
  - Risk: LOW - just documentation

- **`/home/mercleDev/codebase/INTEGRATION_STATUS.txt`**
  - Reason: Old status file, superseded by current docs
  - Size: Text file
  - Risk: LOW - just documentation

- **`/home/mercleDev/codebase/OLD_VS_NEW_COMPARISON.txt`**
  - Reason: Historical comparison, no longer relevant
  - Size: Text file
  - Risk: LOW - just documentation

- **`/home/mercleDev/codebase/TRUSTZONE_IMPLEMENTATION_COMPLETE.txt`**
  - Reason: Implementation complete marker, redundant
  - Size: Text file
  - Risk: LOW - just documentation

- **`/home/mercleDev/codebase/MOBILE_APP_INTEGRATION_SPEC.txt`**
  - Reason: Old spec, may be superseded by MESSAGE_PROTOCOL.md
  - Size: Text file
  - Risk: LOW - just documentation

- **`/home/mercleDev/codebase/DEPLOYMENT_GUIDE.txt`**
  - Reason: Text version, may be redundant with shell scripts
  - Size: Text file
  - Risk: LOW - just documentation

### 2. TEST FILES (Development/Debugging Only)

#### Python Test Scripts
- **`/home/mercleDev/codebase/test_camera_view.py`**
  - Reason: Development test script
  - Purpose: Camera testing during development
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/test_face_simple.py`**
  - Reason: Development test script
  - Purpose: Face detection testing
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/test_face_landmarks_web.py`**
  - Reason: Development test script
  - Purpose: Landmark detection testing
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/test_nose_mediapipe.py`**
  - Reason: Development test script
  - Purpose: Nose tracking testing
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/test_performance_diag.py`**
  - Reason: Development test script
  - Purpose: Performance diagnostics
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/test_websocket.py`**
  - Reason: Development test script
  - Purpose: WebSocket testing
  - Risk: LOW - not needed in production

#### C++ Test Files
- **`/home/mercleDev/codebase/test_device_key.cpp`**
  - Reason: Test program for device key
  - Purpose: Development testing
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/test_device_key`** (compiled binary)
  - Reason: Compiled test binary
  - Purpose: Testing executable
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/test_qr_challenge.cpp`**
  - Reason: Test program for QR challenges
  - Purpose: Development testing
  - Risk: LOW - not needed in production

### 3. EC2 SERVER TEST FILES

- **`/home/mercleDev/codebase/ec2-server/quick_test.py`**
  - Reason: Quick test script for server
  - Purpose: Development testing
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/ec2-server/test_full_flow.py`**
  - Reason: Full flow test script
  - Purpose: Development testing
  - Risk: LOW - not needed in production

- **`/home/mercleDev/codebase/ec2-server/test_production.py`**
  - Reason: Production test script (may want to keep)
  - Purpose: Production testing
  - Risk: MEDIUM - might be useful for validation

### 4. BACKUP FILES

- **`/home/mercleDev/codebase/lib/mediapipe/libface_mesh_wrapper.so.x86_64.backup`**
  - Reason: Backup of x86_64 library (wrong architecture for ARM)
  - Purpose: Old backup from different architecture
  - Risk: LOW - not needed on ARM device

### 5. MEDIAPIPE REBUILD ARTIFACTS

- **`/home/mercleDev/codebase/mediapipe_rebuild/build.log`**
  - Reason: Build log file
  - Purpose: Historical build output
  - Risk: LOW - just a log file

- **`/home/mercleDev/codebase/mediapipe_rebuild/test_wrapper.cpp`**
  - Reason: Test wrapper for mediapipe
  - Purpose: Development testing
  - Risk: LOW - not needed in production

### 6. BUILD OUTPUT LOGS

- **`/home/mercleDev/codebase/build/build_output.log`**
  - Reason: CMake build output log
  - Purpose: Build log
  - Risk: LOW - regenerated on each build

- **`/home/mercleDev/codebase/build/cmake_output.log`**
  - Reason: CMake output log
  - Purpose: Build log
  - Risk: LOW - regenerated on each build

### 7. DEVICE-SPECIFIC FILES (May be specific to test device)

- **`/home/mercleDev/codebase/device_label_b4_2f_03_31_9a_35.png`**
  - Reason: Device label for specific test device
  - Purpose: Device identification
  - Risk: MEDIUM - specific to one device

- **`/home/mercleDev/codebase/device_label_b4_2f_03_31_9a_35.txt`**
  - Reason: Device label text for specific test device
  - Purpose: Device identification
  - Risk: MEDIUM - specific to one device

---

## 🟡 MEDIUM CONFIDENCE - Review Carefully

### Documentation That Might Be Useful

- **`/home/mercleDev/codebase/TODO_CHECKLIST.md`**
  - Reason: May have pending tasks
  - Purpose: Track TODO items
  - Risk: MEDIUM - might have important notes
  - **ACTION: Review contents first**

- **`/home/mercleDev/codebase/QR_ENCRYPTION_ANALYSIS.md`**
  - Reason: Technical analysis document
  - Purpose: QR encryption documentation
  - Risk: MEDIUM - might be useful reference
  - **ACTION: Review if contains unique info**

### Example Files

- **`/home/mercleDev/codebase/examples/simple_liveness_viewer.cpp`**
  - Reason: Example program
  - Purpose: Demonstration/reference
  - Risk: MEDIUM - might be useful for future reference
  - **ACTION: Keep if you want examples**

### EC2 Server Simulation Scripts

- **`/home/mercleDev/codebase/ec2-server/simulate_mobile.py`**
  - Reason: Mobile simulator for testing
  - Purpose: Development/testing
  - Risk: MEDIUM - useful for testing without mobile app
  - **ACTION: Keep if you need to test without real mobile**

- **`/home/mercleDev/codebase/ec2-server/simulate_rdk.py`**
  - Reason: RDK simulator for testing
  - Purpose: Development/testing
  - Risk: MEDIUM - useful for testing without RDK device
  - **ACTION: Keep if you need to test without real device**

### Unused Model Files (Check if actually used)

- **`/home/mercleDev/codebase/models/deploy_ssd.prototxt`**
  - Reason: SSD model (not currently used?)
  - Purpose: Face detection model
  - Risk: MEDIUM - check if code references this

- **`/home/mercleDev/codebase/models/deploy.prototxt`**
  - Reason: Model file (not currently used?)
  - Purpose: Face detection model
  - Risk: MEDIUM - check if code references this

- **`/home/mercleDev/codebase/models/res10_300x300_ssd_iter_140000.caffemodel`**
  - Reason: Caffe model (not currently used?)
  - Purpose: Face detection weights
  - Risk: MEDIUM - check if code references this

- **`/home/mercleDev/codebase/haarcascade_frontalface_alt.xml`**
  - Reason: Haar cascade (old method, not used if using MediaPipe)
  - Purpose: Face detection
  - Risk: MEDIUM - check if code references this

---

## 🟢 LOW RISK - Keep These

### Essential Documentation
- ✅ `BOOT_SEQUENCE_STATUS.md` - Current boot sequence docs
- ✅ `BOOT_SEQUENCE_FLOW_DIAGRAM.txt` - Visual flow
- ✅ `SCREEN_7_PLACEHOLDER_IMPLEMENTATION.md` - Current implementation
- ✅ `COMPLETE_SYSTEM_ARCHITECTURE.md` - System architecture
- ✅ `PRODUCTION_FIXES_SUMMARY.md` - Production fixes
- ✅ `PRODUCTION_READY_STATUS.md` - Production status
- ✅ `QUICK_REFERENCE.md` - Quick reference
- ✅ `EC2_SSH_CREDENTIALS.md` - Important credentials

### Essential Code
- ✅ All files in `/src/` - Source code
- ✅ All files in `/include/` - Headers
- ✅ All files in `/deployment/` - Deployment scripts
- ✅ All files in `/tools/` - Utility tools
- ✅ `CMakeLists.txt` - Build configuration
- ✅ `build-and-run.sh` - Build script
- ✅ `run_liveness_viewer.sh` - Run script
- ✅ `production_ready_check.sh` - Production check

### EC2 Server Essentials
- ✅ `ec2-server/server.py` - Main server
- ✅ `ec2-server/MESSAGE_PROTOCOL.md` - Protocol docs
- ✅ `ec2-server/CONNECTION_DOC.md` - Connection docs
- ✅ `ec2-server/README.md` - Server readme
- ✅ `ec2-server/requirements.txt` - Python deps
- ✅ `ec2-server/user_portal.html` - User portal

### Essential Models
- ✅ `models/face_landmarker.task` - MediaPipe model (USED)
- ✅ `models/yolov5n-face.onnx` - YOLOv5 face model (might be used)

### Build Artifacts (Keep for now, can regenerate)
- ✅ `build/mdai_system` - Main executable
- ✅ `build/libmdai_realsense.so` - Shared library

---

## 🔵 OUTSIDE CODEBASE - Review Separately

### /home/mercleDev/ Root Level Files

These are OUTSIDE the codebase folder and should be reviewed separately:

**Test Scripts (Duplicates?):**
- `/home/mercleDev/test_camera_view.py`
- `/home/mercleDev/test_face_landmarks_web.py`
- `/home/mercleDev/test_face_simple.py`
- `/home/mercleDev/test_landmarks_roi.py`
- `/home/mercleDev/test_landmarks_working.py`
- `/home/mercleDev/test_nose_mediapipe.py`
- `/home/mercleDev/test_nose_tracking_simple.py`
- `/home/mercleDev/test_performance_diag.py`

**Firmware Files:**
- `/home/mercleDev/bootloader.bin`
- `/home/mercleDev/firmware_from_laptop.bin`
- `/home/mercleDev/firmware.bin`
- `/home/mercleDev/partitions.bin`

**Stress Test Scripts:**
- `/home/mercleDev/honest-stress-test.sh`
- `/home/mercleDev/rdk-honest-bpu-test.sh`
- `/home/mercleDev/rdk-max-stress-test.sh`
- `/home/mercleDev/rdk-real-max-power.sh`
- `/home/mercleDev/rdk-stress-test.sh`
- `/home/mercleDev/real-bpu-inference-test.sh`

**Database Files:**
- `/home/mercleDev/mdai_server.db` (duplicate?)
- `/home/mercleDev/flask.log`

**Other:**
- `/home/mercleDev/benchmark_bpu.py`
- `/home/mercleDev/set_static_ip_rdk.sh`
- `/home/mercleDev/node-v24.11.1-linux-arm64.tar.xz` (Node.js installer)
- `/home/mercleDev/mdaiplan.odt` (Your planning document - KEEP!)

---

## 📋 DELETION SUMMARY

### Recommended for Immediate Deletion (LOW RISK)

**Count: 20 files**

1. Old documentation: 7 files
2. Test scripts: 9 files
3. Backup files: 1 file
4. Build logs: 2 files
5. Compiled test binaries: 1 file

### Review Before Deleting (MEDIUM RISK)

**Count: 11 files**

1. Device-specific files: 2 files
2. Documentation to review: 2 files
3. Example files: 1 file
4. Simulation scripts: 2 files
5. Possibly unused models: 4 files

### Total Potential Cleanup

- **Within /home/mercleDev/codebase/**: ~31 files
- **Build artifacts**: Can clean entire `/build/` folder (regenerated on compile)
- **Outside codebase**: ~30+ files (needs separate review)

---

## 🎯 RECOMMENDED ACTION PLAN

### Phase 1: Safe Cleanup (Do Now)
Delete the 20 LOW RISK files listed above.

### Phase 2: Review and Decide (Manual Review)
Review the 11 MEDIUM RISK files and decide if needed.

### Phase 3: Build Folder Cleanup (Optional)
Clean the entire `/build/` folder and rebuild:
```bash
cd /home/mercleDev/codebase
rm -rf build/
mkdir build && cd build
cmake .. && make -j$(nproc)
```

### Phase 4: Home Directory Cleanup (Separate Task)
Review and clean files in `/home/mercleDev/` root.

---

## ✅ READY TO PROCEED?

Please review this list and let me know:
1. ✅ Approve deletion of all LOW RISK files?
2. 🤔 Which MEDIUM RISK files to keep/delete?
3. 🔵 Should I also review home directory files?

Once approved, I'll proceed with the deletion.

