#include "face_mesh_wrapper.h"
#include <Python.h>
#include <numpy/arrayobject.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <mutex>
#include <memory>

namespace mdai {

// Static initialization for Python
static std::once_flag python_init_flag;
static PyObject* bridge_module = nullptr;
static PyObject* process_func = nullptr;
static PyObject* init_func = nullptr;
static PyObject* cleanup_func = nullptr;
static bool python_initialized = false;

// Helper function for NumPy initialization
static int init_numpy() {
    import_array();
    return 0;
}

/**
 * Initialize Python interpreter and import the bridge module
 */
static bool InitializePython() {
    if (python_initialized) return true;
    
    std::call_once(python_init_flag, []() {
        std::cout << "[FaceMeshBridge] Initializing Python..." << std::endl;
        
        Py_Initialize();
        
        if (!Py_IsInitialized()) {
            std::cerr << "[FaceMeshBridge] Failed to initialize Python" << std::endl;
            return;
        }
        
        // Initialize NumPy
        if (init_numpy() < 0) {
            std::cerr << "[FaceMeshBridge] Failed to initialize NumPy" << std::endl;
            return;
        }
        
        // Add the bridge module path to Python path
        PyObject* sys_path = PySys_GetObject("path");
        if (sys_path) {
            PyList_Append(sys_path, PyUnicode_FromString("/home/mercleDev/codebase/lib/mediapipe"));
        }
        
        // Import the bridge module
        bridge_module = PyImport_ImportModule("face_mesh_bridge");
        if (!bridge_module) {
            PyErr_Print();
            std::cerr << "[FaceMeshBridge] Failed to import face_mesh_bridge module" << std::endl;
            std::cerr << "[FaceMeshBridge] Make sure face_mesh_bridge.py is in /home/mercleDev/codebase/lib/mediapipe/" << std::endl;
            return;
        }
        
        // Get the functions
        init_func = PyObject_GetAttrString(bridge_module, "initialize");
        process_func = PyObject_GetAttrString(bridge_module, "process_image");
        cleanup_func = PyObject_GetAttrString(bridge_module, "cleanup");
        
        if (!init_func || !process_func || !cleanup_func) {
            PyErr_Print();
            std::cerr << "[FaceMeshBridge] Failed to get bridge functions" << std::endl;
            return;
        }
        
        // Call initialize()
        PyObject* init_result = PyObject_CallObject(init_func, nullptr);
        if (!init_result || init_result == Py_False) {
            PyErr_Print();
            std::cerr << "[FaceMeshBridge] Failed to initialize MediaPipe bridge" << std::endl;
            Py_XDECREF(init_result);
            return;
        }
        Py_DECREF(init_result);
        
        python_initialized = true;
        std::cout << "[FaceMeshBridge] Python bridge initialized successfully" << std::endl;
    });
    
    return python_initialized;
}

/**
 * Convert OpenCV Mat to NumPy array
 */
static PyObject* Mat2NumPy(const cv::Mat& image) {
    npy_intp dimensions[3] = {image.rows, image.cols, image.channels()};
    
    PyObject* array = PyArray_SimpleNew(3, dimensions, NPY_UINT8);
    if (!array) {
        PyErr_Print();
        return nullptr;
    }
    
    uint8_t* dst = (uint8_t*)PyArray_DATA((PyArrayObject*)array);
    memcpy(dst, image.data, image.total() * image.elemSize());
    
    return array;
}

// ============================================================================
// FaceMeshDetector::Impl - PIMPL Implementation using Python Bridge
// ============================================================================

class FaceMeshDetector::Impl {
public:
    Impl(const FaceMeshConfig& config) 
        : config_(config)
        , initialized_(false)
        , last_error_("Not initialized")
    {
        Initialize();
    }
    
    ~Impl() {
        // Note: Don't cleanup Python here as it may be shared
    }
    
    bool Initialize() {
        if (initialized_) return true;
        
        if (!InitializePython()) {
            last_error_ = "Failed to initialize Python bridge";
            return false;
        }
        
        initialized_ = true;
        last_error_ = "";
        std::cout << "[FaceMeshBridge] Detector ready" << std::endl;
        return true;
    }
    
    bool Detect(const cv::Mat& bgr_image, FaceMeshResult& result) {
        if (!initialized_) {
            last_error_ = "Detector not initialized";
            return false;
        }
        
        if (bgr_image.empty()) {
            last_error_ = "Empty input image";
            return false;
        }
        
        // Convert BGR to RGB
        cv::Mat rgb_image;
        cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
        
        // Acquire GIL
        PyGILState_STATE gstate = PyGILState_Ensure();
        
        // Convert to NumPy
        PyObject* numpy_image = Mat2NumPy(rgb_image);
        if (!numpy_image) {
            last_error_ = "Failed to convert image to NumPy";
            PyGILState_Release(gstate);
            return false;
        }
        
        // Call process_image(numpy_image)
        PyObject* args = PyTuple_Pack(1, numpy_image);
        PyObject* result_dict = PyObject_CallObject(process_func, args);
        Py_DECREF(args);
        Py_DECREF(numpy_image);
        
        if (!result_dict) {
            PyErr_Print();
            last_error_ = "Bridge process call failed";
            PyGILState_Release(gstate);
            return false;
        }
        
        // Parse the result
        bool success = ParseResult(result_dict, rgb_image.cols, rgb_image.rows, result);
        Py_DECREF(result_dict);
        
        PyGILState_Release(gstate);
        return success;
    }
    
    bool IsInitialized() const { return initialized_; }
    std::string GetLastError() const { return last_error_; }
    
private:
    bool ParseResult(PyObject* result_dict, int width, int height, FaceMeshResult& result) {
        // Clear previous results
        result.landmarks.clear();
        result.visibility.clear();
        result.presence.clear();
        result.confidence = 0.0f;
        result.image_width = width;
        result.image_height = height;
        
        // None means no face detected
        if (result_dict == Py_None) {
            last_error_ = "No face detected";
            return true;  // Not an error, just no face
        }
        
        // Must be a dict
        if (!PyDict_Check(result_dict)) {
            last_error_ = "Result is not a dict";
            return false;
        }
        
        // Get landmarks list
        PyObject* landmarks_list = PyDict_GetItemString(result_dict, "landmarks");
        if (!landmarks_list || !PyList_Check(landmarks_list)) {
            last_error_ = "No landmarks in result";
            return true;  // No face
        }
        
        Py_ssize_t num_landmarks = PyList_Size(landmarks_list);
        if (num_landmarks <= 0) {
            last_error_ = "Empty landmarks list";
            return true;
        }
        
        // Reserve space
        result.landmarks.reserve(num_landmarks);
        result.visibility.reserve(num_landmarks);
        result.presence.reserve(num_landmarks);
        
        float min_x = 1.0f, min_y = 1.0f, max_x = 0.0f, max_y = 0.0f;
        
        // Parse each landmark (now it's a simple Python list of dicts!)
        for (Py_ssize_t i = 0; i < num_landmarks; i++) {
            PyObject* lm_dict = PyList_GetItem(landmarks_list, i);  // Borrowed reference
            if (!lm_dict || !PyDict_Check(lm_dict)) {
                continue;
            }
            
            // Get x, y, z from dict
            PyObject* x_obj = PyDict_GetItemString(lm_dict, "x");
            PyObject* y_obj = PyDict_GetItemString(lm_dict, "y");
            PyObject* z_obj = PyDict_GetItemString(lm_dict, "z");
            PyObject* vis_obj = PyDict_GetItemString(lm_dict, "visibility");
            PyObject* pres_obj = PyDict_GetItemString(lm_dict, "presence");
            
            float x_norm = x_obj ? (float)PyFloat_AsDouble(x_obj) : 0.0f;
            float y_norm = y_obj ? (float)PyFloat_AsDouble(y_obj) : 0.0f;
            float z_norm = z_obj ? (float)PyFloat_AsDouble(z_obj) : 0.0f;
            
            // Convert to pixel coordinates
            float x_pix = x_norm * width;
            float y_pix = y_norm * height;
            float z_pix = z_norm * width;
            
            result.landmarks.push_back(cv::Point3f(x_pix, y_pix, z_pix));
            
            float visibility = vis_obj ? (float)PyFloat_AsDouble(vis_obj) : 1.0f;
            float presence = pres_obj ? (float)PyFloat_AsDouble(pres_obj) : 1.0f;
            
            result.visibility.push_back(visibility);
            result.presence.push_back(presence);
            
            // Track bounding box
            if (x_norm < min_x) min_x = x_norm;
            if (x_norm > max_x) max_x = x_norm;
            if (y_norm < min_y) min_y = y_norm;
            if (y_norm > max_y) max_y = y_norm;
        }
        
        // Set bounding box
        result.bbox = cv::Rect(
            static_cast<int>(min_x * width),
            static_cast<int>(min_y * height),
            static_cast<int>((max_x - min_x) * width),
            static_cast<int>((max_y - min_y) * height)
        );
        
        // Get confidence
        PyObject* conf_obj = PyDict_GetItemString(result_dict, "confidence");
        result.confidence = conf_obj ? (float)PyFloat_AsDouble(conf_obj) : 0.0f;
        
        last_error_ = "";
        return true;
    }
    
    FaceMeshConfig config_;
    bool initialized_;
    std::string last_error_;
};

// ============================================================================
// FaceMeshResult Implementation
// ============================================================================

bool FaceMeshResult::IsOccluded(float visibility_threshold, float occlusion_ratio) const {
    if (visibility.empty()) return false;
    
    int occluded_count = 0;
    for (float vis : visibility) {
        if (vis < visibility_threshold) {
            occluded_count++;
        }
    }
    
    float ratio = static_cast<float>(occluded_count) / visibility.size();
    return ratio > occlusion_ratio;
}

std::vector<cv::Point3f> FaceMeshResult::GetRegionLandmarks(const std::vector<int>& indices) const {
    std::vector<cv::Point3f> region;
    region.reserve(indices.size());
    
    for (int idx : indices) {
        if (idx >= 0 && idx < static_cast<int>(landmarks.size())) {
            region.push_back(landmarks[idx]);
        }
    }
    
    return region;
}

bool FaceMeshResult::IsLandmarkValid(int index, float min_visibility) const {
    if (index < 0 || index >= static_cast<int>(landmarks.size())) {
        return false;
    }
    
    if (index >= static_cast<int>(visibility.size())) {
        return true;
    }
    
    return visibility[index] >= min_visibility;
}

// Landmark indices
const std::vector<int> FaceMeshResult::LandmarkIndices::FACE_OVAL = {
    10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
    397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
    172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LEFT_EYE = {
    33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246
};

const std::vector<int> FaceMeshResult::LandmarkIndices::RIGHT_EYE = {
    263, 249, 390, 373, 374, 380, 381, 382, 362, 398, 384, 385, 386, 387, 388, 466
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LIPS_OUTER = {
    61, 185, 40, 39, 37, 0, 267, 269, 270, 409, 291, 308
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LIPS_INNER = {
    78, 191, 80, 81, 82, 13, 312, 311, 310, 415, 308, 324
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LEFT_EYEBROW = {
    46, 53, 52, 65, 55, 70, 63, 105
};

const std::vector<int> FaceMeshResult::LandmarkIndices::RIGHT_EYEBROW = {
    276, 283, 282, 295, 285, 300, 293, 334
};

const std::vector<int> FaceMeshResult::LandmarkIndices::NOSE = {
    1, 2, 98, 327, 4, 5, 6, 168, 197
};

const std::vector<int> FaceMeshResult::LandmarkIndices::FOREHEAD = {
    10, 338, 297, 332, 284, 251, 389, 356, 454, 323
};

const std::vector<int> FaceMeshResult::LandmarkIndices::LEFT_CHEEK = {
    50, 117, 118, 119, 120, 121, 122, 123, 126, 203
};

const std::vector<int> FaceMeshResult::LandmarkIndices::RIGHT_CHEEK = {
    280, 346, 347, 348, 349, 350, 351, 352, 355, 423
};

// ============================================================================
// FaceMeshDetector Implementation
// ============================================================================

FaceMeshDetector::FaceMeshDetector(const FaceMeshConfig& config) 
    : impl_(std::make_unique<Impl>(config))
{
}

FaceMeshDetector::~FaceMeshDetector() = default;

bool FaceMeshDetector::Detect(const cv::Mat& rgb_image, FaceMeshResult& result) {
    return impl_->Detect(rgb_image, result);
}

bool FaceMeshDetector::IsInitialized() const {
    return impl_->IsInitialized();
}

std::string FaceMeshDetector::GetLastError() const {
    return impl_->GetLastError();
}

}  // namespace mdai
